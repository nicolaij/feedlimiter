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

#include "tm1637.h"

#define BLOCK_SIZE 400 * 2

extern int setup_current_changed;

static TaskHandle_t s_task_handle;

extern QueueHandle_t xQueueDisplay;

extern int key_dir;

extern int parameters_changed;

adc_continuous_handle_t adc_handle = NULL;
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

    dac_oneshot_config_t chan1_cfg = {
        .chan_id = DAC_CHAN_0,
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

#define OPERATE (100 / 20)

    int operate_count = 0;
    int operate_sum = 0;

    vTaskDelay(1);

    float k_calc = get_menu_val_by_id("Kcalc");
    float setup_current = get_menu_val_by_id("elcurrent");

    pid_ctrl_block_handle_t pid_handle;

    pid_ctrl_config_t pid_conf = {.init_param.cal_type = PID_CAL_TYPE_POSITIONAL,
                                  .init_param.kp = get_menu_val_by_id("pidP"),
                                  .init_param.ki = get_menu_val_by_id("pidI"),
                                  .init_param.kd = get_menu_val_by_id("pidD"),
                                  .init_param.max_integral = get_menu_val_by_id("pidintMax"),
                                  .init_param.min_integral = get_menu_val_by_id("pidintMin"),
                                  .init_param.max_output = get_menu_val_by_id("pidMax"),
                                  .init_param.min_output = get_menu_val_by_id("pidMin")};

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

            float avgo = sqrtf((float)operate_sum / OPERATE);
            operate_count = 0;
            operate_sum = 0;

            if (!current_offset_set)
            {
                current_offset = avgo;
                current_offset_set = true;

                ESP_LOGI("main", "ADC0 offset: %f", avgo);

                continue;
            }

            switch (parameters_changed)
            {
            case 1: // задание
                setup_current = get_menu_val_by_id("elcurrent");
                /* code */
                break;
            case 4: // P
            case 5: // I
            case 6: // D
            case 7: // Out max
            case 8: // Out min
                pid_ctrl_parameter_t params = {.cal_type = PID_CAL_TYPE_POSITIONAL,
                                               .kp = get_menu_val_by_id("pidP"),
                                               .ki = get_menu_val_by_id("pidI"),
                                               .kd = get_menu_val_by_id("pidD"),
                                               .max_integral = get_menu_val_by_id("pidintMax"),
                                               .min_integral = get_menu_val_by_id("pidintMin"),
                                               .max_output = get_menu_val_by_id("pidMax"),
                                               .min_output = get_menu_val_by_id("pidMin")};

                pid_update_parameters(pid_handle, &params);
                /* code */
                break;

            default:
                break;
            }

            parameters_changed = 0;

            float current = avgo * k_calc; // in A
                                           // ESP_LOGD("main", "Current: %4.1f A  %4.1f * %d", current, avgo, k_calc);

            if (key_dir == 0)
                xQueueSend(xQueueDisplay, &current, 0);

            uint8_t dac1 = UINT8_MAX;
            static float ret_result = 0;
            float input_error = setup_current - current;
            // float intg = pid_handle->integral_err;
            ESP_ERROR_CHECK(pid_compute(pid_handle, input_error, &ret_result));
            if ((int)ret_result < UINT8_MAX)
            {
                dac1 = (int)ret_result;
            }

            dac_oneshot_output_voltage(chan1_handle, dac1);

            ESP_LOGI("main", "Current: %4.1f A (%4.0f ); PID: %4.3f, %.0f + %.0f + %.0f", current, avgo, ret_result, input_error * pid_handle->Kp, pid_handle->integral_err * pid_handle->Ki, (input_error - pid_handle->previous_err2) * pid_handle->Kd);
        }
    }
}

void displ_task(void *arg)
{

    // Configure the display
    tm1637_config_t config = {
        .clk_pin = TM1637_CLK_PIN,
        .dio_pin = TM1637_DIO_PIN,
        .bit_delay_us = 100 // Default timing
    };

    // Initialize the display
    tm1637_handle_t display;
    esp_err_t ret = tm1637_init(&config, &display);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize TM1637: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Display initialized successfully");

    // Set brightness to medium (0-7 range)
    tm1637_set_brightness(display, 4, true);

    // Example 1: Display a number
    ESP_LOGI(TAG, "Displaying number: 1234");
    tm1637_show_number(display, 1234, false, 4, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    float val = 0;
    uint8_t char_display[4];
    int val1 = 99;

    while (1)
    {
        if (xQueueReceive(xQueueDisplay, &val, 500 / portTICK_PERIOD_MS))
        {
            val1 = val;
            if (val1 > 99)
                val1 = 99;
            if (val1 < 0)
                val1 = 0;

            char_display[0] = (val1 >= 10.0) ? tm1637_encode_digit(val1 / 10) : 0;
            char_display[1] = tm1637_encode_digit(val1 % 10) | TM1637_SEG_DP;
            char_display[2] = 0;
            char_display[3] = 0;

            tm1637_set_segments(display, char_display, 4, 0);
        }
    }
}