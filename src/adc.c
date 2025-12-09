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

#include "hal/adc_ll.h"

#define POINTS10 100 // измерений каждых 10 мс
#define CYCLE 300    // ms

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
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init()
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = POINTS10 * 2 * 2 * 2 * 2,
        .conv_frame_size = POINTS10 * 2 * 2,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = 2;

    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = ADC_CHANNEL;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_pattern[1].atten = ADC_ATTEN_DB_12;
    adc_pattern[1].channel = ADC_CHANNEL_0;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

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
    uint8_t result[POINTS10 * 2 * 2] = {0};
    int median_filter_current[3];
    uint8_t median_filter_current_fill = 0;
    int median_filter_setup[3];
    uint8_t median_filter_setup_fill = 0;
    int digital_filter;

    int current_offset = 0;
    bool current_offset_set = false;

    int operate_count = 0;
    int operate_sum = 0;

    int avg_setup_sum = 0;

    int run_stage = 0;

    float currents[UINT8_MAX];
    uint8_t currents_cnt = 0;

    float current_xx = 0;

    int err_count = 0;

    vTaskDelay(1);

        gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = BIT64(GPIO_NUM_15);
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));


    s_task_handle = xTaskGetCurrentTaskHandle();

    float k_calc = get_menu_val_by_id("Kcalc");
    float Isetmin = get_menu_val_by_id("Isetmin");
    float Isetmax = get_menu_val_by_id("Isetmax");
    float Iporog = get_menu_val_by_id("Iporog");

    float ADCmax = get_menu_val_by_id("ADCmax");

    // float setup_current = get_menu_val_by_id("elcurrent");

    displ_t displ_data;

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
    //adc_ll_digi_set_convert_limit_num(2);

    int64_t time1 = esp_timer_get_time();
    int64_t time2 = esp_timer_get_time();

    while (1)
    {
        // time1 = esp_timer_get_time();

        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        adc_digi_output_data_t *p = (void *)result;
        int sum_current = 0;
        int count_current = 0;
        int sum_setup = 0;
        int count_setup = 0;

        ret = adc_continuous_read(adc_handle, result, POINTS10 * 2 * 2, &ret_num, ADC_MAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT)
        {
            vTaskDelay(1);
            continue;
        }

        while ((uint8_t *)p < result + ret_num)
        {
            switch (p->type1.channel)
            {
            case ADC_CHANNEL:
                median_filter_current[median_filter_current_fill % 3] = p->type1.data;
                median_filter_current_fill++;

                if (median_filter_current_fill >= 3)
                {
                    digital_filter = MEDIAN(median_filter_current);
                    if (median_filter_current_fill >= 6)
                        median_filter_current_fill = 3;
                }
                else
                {
                    digital_filter = p->type1.data;
                }

                sum_current += digital_filter;
                count_current++;
                break;

            case ADC_CHANNEL_0:
                median_filter_setup[median_filter_setup_fill % 3] = p->type1.data;
                median_filter_setup_fill++;

                if (median_filter_setup_fill >= 3)
                {
                    digital_filter = MEDIAN(median_filter_setup);
                    if (median_filter_setup_fill >= 6)
                        median_filter_setup_fill = 3;
                }
                else
                {
                    digital_filter = p->type1.data;
                }

                sum_setup += digital_filter;
                count_setup++;
            default:
                err_count++;
                break;
            }
            p++;
        }

        // time2 = esp_timer_get_time();
        // ESP_LOGD("main", "time: %8lld; cnt: %d; ret: %d, %x", time2 - time1, count_current, ret_num, ret);
        // continue;

        // обработка 
        if (count_current > 0)
        {
            int avg = sum_current / count_current - current_offset;
            int avg_setup = sum_setup / count_setup;

            operate_sum += avg * avg;
            avg_setup_sum += avg_setup;
            operate_count++;

            if (operate_count < (CYCLE / 10))
            {
                continue;
            }

            float avgo = sqrtf((float)operate_sum / operate_count);
            float avgs = avg_setup_sum / operate_count;

            operate_count = 0;
            operate_sum = 0;
            avg_setup_sum = 0;

            if (!current_offset_set)
            {
                current_offset = avgo;
                current_offset_set = true;

                ESP_LOGI("main", "ADC0 offset: %f", avgo);

                run_stage = 1;
                continue;
            }

            if (parameters_changed)
            {
                pid_ctrl_parameter_t params = {.cal_type = PID_CAL_TYPE_POSITIONAL,
                                               .kp = get_menu_val_by_id("pidP"),
                                               .ki = get_menu_val_by_id("pidI"),
                                               .kd = get_menu_val_by_id("pidD"),
                                               .max_integral = get_menu_val_by_id("pidintMax"),
                                               .min_integral = get_menu_val_by_id("pidintMin"),
                                               .max_output = get_menu_val_by_id("pidMax"),
                                               .min_output = get_menu_val_by_id("pidMin")};

                pid_update_parameters(pid_handle, &params);

                k_calc = get_menu_val_by_id("Kcalc");
                Isetmin = get_menu_val_by_id("Isetmin");
                Isetmax = get_menu_val_by_id("Isetmax");
                Iporog = get_menu_val_by_id("Iporog");
                ADCmax = get_menu_val_by_id("ADCmax");
                parameters_changed = 0;
            }

            float current = avgo * k_calc; // in A

            currents[currents_cnt] = current;

            if (run_stage == 5)
            {
                if (currents[currents_cnt] < (current_xx + 1.0) && currents[(uint8_t)(currents_cnt - 1)] < (current_xx + 1.0) && currents[(uint8_t)(currents_cnt - 2)] < (current_xx + 1.0) && currents[(uint8_t)(currents_cnt - 3)] < (current_xx + 1.0) && currents[(uint8_t)(currents_cnt - 4)] < (current_xx + 1.0) && currents[(uint8_t)(currents_cnt - 5)] < (current_xx + 1.0))
                {
                    run_stage = 4;
                }
            }

            if (run_stage == 4) // врезка
            {
                if (current > current_xx + 2.0)
                {
                    run_stage = 5;

                    pid_handle->integral_err = (avgs / ADCmax * UINT8_MAX) / pid_handle->Ki;
                }

                if (currents[currents_cnt] < (2.0) && currents[(uint8_t)(currents_cnt - 1)] < (2.0) && currents[(uint8_t)(currents_cnt - 2)] < (2.0))
                {
                    run_stage = 1;
                }
            }

            if (run_stage == 3) // фиксируем ХХ
            {
                current_xx = (currents[currents_cnt] + currents[(uint8_t)(currents_cnt - 1)] + currents[(uint8_t)(currents_cnt - 2)]) / 3;
                Isetmin = current_xx;
                Iporog = current_xx + 2;
                run_stage = 4;
            }

            if (run_stage == 2) // ждем стабилизации тока
            {
                if (currents[currents_cnt] <= currents[(uint8_t)(currents_cnt - 1)] + 1 && currents[currents_cnt] >= currents[(uint8_t)(currents_cnt - 1)] - 1 && currents[currents_cnt] <= currents[(uint8_t)(currents_cnt - 2)] + 1 && currents[currents_cnt] >= currents[(uint8_t)(currents_cnt - 2)] - 1 && currents[currents_cnt] <= currents[(uint8_t)(currents_cnt - 3)] + 1 && currents[currents_cnt] >= currents[(uint8_t)(currents_cnt - 3)] - 1)
                {
                    if (currents[currents_cnt] < Iporog)
                        run_stage = 3;
                }
            }

            currents_cnt++;

            if (run_stage == 1) // ловим пуск пилы
            {
                if (current > 50.0)
                    run_stage = 2;
            }

            if (gpio_get_level(GPIO_NUM_15) == 1)
                run_stage = 1;

            float setup_current = (Isetmin + avgs / ADCmax * (Isetmax - Isetmin));
            if (setup_current < Iporog)
                setup_current = Iporog;

            displ_data.curr = current;
            displ_data.dots = true;
            displ_data.set = setup_current;

            // ESP_LOGD("main", "Current: %4.1f A  %4.1f * %d", current, avgo, k_calc);

            xQueueSend(xQueueDisplay, &displ_data, 0);

            uint8_t dac1 = UINT8_MAX;
            static float ret_result = 0;
            float input_error = setup_current - current;
            // float intg = pid_handle->integral_err;
            pid_handle->max_output = avgs / ADCmax * UINT8_MAX * 1.1;
            ESP_ERROR_CHECK(pid_compute(pid_handle, input_error, &ret_result));

            dac1 = (int)ret_result;

            if ((int)ret_result > UINT8_MAX)
            {
                dac1 = UINT8_MAX;
            }

            if (run_stage != 5)
            {
                dac1 = avgs / ADCmax * UINT8_MAX;
                pid_reset_ctrl_block(pid_handle);
            }

            dac_oneshot_output_voltage(chan1_handle, dac1);

            time2 = esp_timer_get_time();

            ESP_LOGI("main", "%d stage: %d %8lld Current: %4.1f A (%4.0f) Setup ADC: %4d; DAC: %4d, %.0f + %.0f + %.0f", gpio_get_level(GPIO_NUM_15), run_stage, time2 - time1, current, avgo, avg_setup, dac1, input_error * pid_handle->Kp, pid_handle->integral_err * pid_handle->Ki, (input_error - pid_handle->previous_err2) * pid_handle->Kd);
            printf(">PV:%.1f\n>SP:%.1f\n>MV:%d\n", current, setup_current, dac1);

            time1 = time2;
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

    // Set brightness to medium (0-7", .izm = "", .val range)
    tm1637_set_brightness(display, 4, true);

    displ_t displ_data;
    uint8_t char_display[TM1637_MAX_DIGITS];

    while (1)
    {
        if (xQueueReceive(xQueueDisplay, &displ_data, portMAX_DELAY))
        {
            int val1 = displ_data.curr;
            if (val1 > 99)
                val1 = 99;
            if (val1 < 0)
                val1 = 0;

            int val2 = displ_data.set;
            if (val2 > 99)
                val2 = 99;
            if (val2 < 0)
                val2 = 0;

            char_display[0] = (val1 >= 10.0) ? tm1637_encode_digit(val1 / 10) : 0;
            char_display[1] = tm1637_encode_digit(val1 % 10) | ((displ_data.dots == true) ? TM1637_SEG_DP : 0);
            char_display[2] = tm1637_encode_digit(val2 / 10);
            char_display[3] = tm1637_encode_digit(val2 % 10);

            tm1637_set_segments(display, char_display, 4, 0);
        }
        vTaskDelay(1);
    }
}