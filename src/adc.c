#include "main.h"
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_partition.h"

#include "driver/gpio.h"

#include "esp_timer.h"

#include "esp_rom_sys.h"

#include "esp_wifi.h"

#include <math.h>
#include "esp_adc/adc_continuous.h"

#include "driver/dac_oneshot.h"

#include "pid_ctrl.h"

#define BLOCK_SIZE 400 * 2
#define ADC_CHANNEL ADC_CHANNEL_7

float dac0_k = 1.0 / 16.0;
float dac1_k = 1.0 / 16.0;
float setup = 50.0;

static TaskHandle_t s_task_handle;

adc_continuous_handle_t adc_handle = NULL;
dac_oneshot_handle_t chan0_handle;
dac_oneshot_handle_t chan1_handle;

static const char *TAG = "adc";

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MEDIAN(a) (MAX(a[0], a[1]) == MAX(a[1], a[2])) ? MAX(a[0], a[2]) : MAX(a[1], MIN(a[0], a[2]))

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t adc_handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    // vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init()
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = BLOCK_SIZE * 2,
        .conv_frame_size = BLOCK_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = 1;

    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = ADC_CHANNEL;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

    /* DAC oneshot init */
    dac_oneshot_config_t chan0_cfg = {
        .chan_id = DAC_CHAN_0,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan0_cfg, &chan0_handle));

    dac_oneshot_config_t chan1_cfg = {
        .chan_id = DAC_CHAN_1,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan1_cfg, &chan1_handle));
}

void adc_task(void *arg)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[BLOCK_SIZE] = {0};
    int median_filter[3];
    uint8_t median_filter_fill = 0;
    int digital_filter;

    int current_offset = 0;
    bool current_offset_set = false;

#define OPERATE (500 / 20)
    int operate_count = 0;
    int operate_sum = 0;

    pid_ctrl_block_handle_t pid_handle;
    pid_ctrl_config_t pid_conf = {.init_param.cal_type = PID_CAL_TYPE_POSITIONAL,
                                  .init_param.kp = 0.01,
                                  .init_param.ki = 0.5,
                                  .init_param.kd = 0,
                                  .init_param.max_integral = 128,
                                  .init_param.min_integral = -128,
                                  .init_param.max_output = 255,
                                  .init_param.min_output = 0};

    continuous_adc_init();

    ESP_ERROR_CHECK(pid_new_control_block(&pid_conf, &pid_handle));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    while (1)
    {
        ret = adc_continuous_read(adc_handle, result, BLOCK_SIZE, &ret_num, ADC_MAX_DELAY);

        adc_digi_output_data_t *p = (void *)result;
        int sum = 0;
        int count = 0;
        while ((uint8_t *)p < result + BLOCK_SIZE)
        {
            if (p->type1.channel == ADC_CHANNEL)
            {
                median_filter[median_filter_fill % 3] = p->type1.data;
                median_filter_fill++;

                if (median_filter_fill >= 3)
                {
                    digital_filter = MEDIAN(median_filter);
                    if (median_filter_fill >= 6)
                        median_filter_fill = 3;
                }
                else
                {
                    digital_filter = p->type1.data;
                }

                sum += digital_filter;
                count++;
            }
            p++;
        }

        // обработка
        if (count > 0)
        {

            int avg = sum / count - current_offset;

            if (operate_count++ < OPERATE)
            {
                operate_sum += avg * avg;
                continue;
            }

            double avgo = sqrt((double)operate_sum / OPERATE);
            operate_count = 0;
            operate_sum = 0;

            if (!current_offset_set)
            {
                current_offset = avgo;
                current_offset_set = true;

                ESP_LOGI("main", "ADC0 offset: %f", avgo);

                continue;
            }



            uint8_t dac0 = UINT8_MAX;
            float d0 = avgo * dac0_k;
            if ((int)d0 < UINT8_MAX)
            {
                dac0 = (int)d0;
            }

            dac_oneshot_output_voltage(chan0_handle, dac0);

            uint8_t dac1 = UINT8_MAX;
            float ret_result = 0;
            float input_error = setup * 4095 / 100.0 - avgo;
            ESP_ERROR_CHECK(pid_compute(pid_handle, input_error, &ret_result));

            // dac_oneshot_output_voltage(chan1_handle, dac1);

            ESP_LOGI("main", "ADC0: %4.1f; DAC0: %4d; PID: %4.3f", avgo, dac0, ret_result);
        }
    }
}
