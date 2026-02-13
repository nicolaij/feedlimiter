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
#include <ht16k33.h>

#include "i2cdev.h"

#include "hal/adc_ll.h"

#define POINTS10 200 // измерений на канал каждых 20 мс
#define CYCLE 200    // ms

extern int setup_current_changed;

static TaskHandle_t s_task_handle;

extern QueueHandle_t xQueueDisplay;

extern int key_mode;

extern int parameters_changed;

adc_continuous_handle_t adc_handle = NULL;

dac_oneshot_handle_t chan1_handle;
dac_oneshot_handle_t chan2_handle;

int run_stage = 1;

static const char *TAG = "adc";

#define MEDIAN(a) (MAX(a[0], a[1]) == MAX(a[1], a[2])) ? MAX(a[0], a[2]) : MAX(a[1], MIN(a[0], a[2]))
#define INRANGE(value, min, max) (value < max) && (value > min)

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
        .max_store_buf_size = POINTS10 * 2 * SOC_ADC_DIGI_RESULT_BYTES * 2,
        .conv_frame_size = POINTS10 * 2 * SOC_ADC_DIGI_RESULT_BYTES,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20000 * 3,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = 2;

    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = ADC_CHANNEL_CURRENT;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_pattern[1].atten = ADC_ATTEN_DB_12;
    adc_pattern[1].channel = ADC_CHANNEL_SET;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

    dac_oneshot_config_t chan1_cfg = {
        .chan_id = DAC_CHAN_0,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan1_cfg, &chan1_handle));

    dac_oneshot_config_t chan2_cfg = {
        .chan_id = DAC_CHAN_1,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&chan2_cfg, &chan2_handle));
}

void adc_task(void *arg)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[POINTS10 * 2 * SOC_ADC_DIGI_RESULT_BYTES] = {0};
    int median_filter_current[3];
    uint8_t median_filter_current_fill = 0;
    int median_filter_setup[3];
    uint8_t median_filter_setup_fill = 0;
    int digital_filter;

    int current_offset = 0;
    int current_offset_set = 5;
    float current_offset_sum = 0;

    int cycle_count = 0;
    int current_sum_rms = 0;
    int current_sum = 0;

    int avg_setup_sum = 0;

    int currents[UINT8_MAX + 1]; // in mA
    uint8_t currents_cnt = 0;

    int current_xx = 0; // in mA

    int err_count = 0;

    int sw1 = 0;

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BIT64(FEED_FORWARD_PIN) | BIT64(DISABLE_PIN);
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    s_task_handle = xTaskGetCurrentTaskHandle();

    float k_calc = get_menu_val_by_id("Kcalc");
    float k_display = get_menu_val_by_id("Kdispl");
    float Isetmin = get_menu_val_by_id("Isetmin");
    float Isetmax = get_menu_val_by_id("Isetmax");
    // float Iporog = get_menu_val_by_id("Iporog");
    float ADCmax = get_menu_val_by_id("ADCmax");

    current_offset = get_menu_val_by_id("offsetADC");

    float Imin = get_menu_val_by_id("Imin");
    float Imax = get_menu_val_by_id("Imax");
    float Iconst = get_menu_val_by_id("Iconst");
    // float Iporog = Isetmin + Iconst + 1.0f;

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

    // ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));

    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    adc_ll_digi_set_convert_limit_num(2);

    int64_t time1 = esp_timer_get_time();
    int64_t time2 = esp_timer_get_time();

    float input_error = 0;

    while (1)
    {
        // time1 = esp_timer_get_time();

        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        adc_digi_output_data_t *p = (void *)result;
        int current_adc_square = 0;
        int current_adc_sum = 0;
        int current_adc_count = 0;
        int setup_adc_sum = 0;
        int setup_adc_count = 0;

        ret = adc_continuous_read(adc_handle, result, POINTS10 * 2 * SOC_ADC_DIGI_RESULT_BYTES, &ret_num, ADC_MAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT)
        {
            vTaskDelay(1);
            continue;
        }

        if (ret == ESP_ERR_INVALID_STATE)
        {
            dac_oneshot_output_voltage(chan1_handle, 0); // stop
            dac_oneshot_output_voltage(chan2_handle, 0);
            vTaskDelay(portMAX_DELAY);
            continue;
        }

        while ((uint8_t *)p < result + ret_num)
        {
            switch (p->type1.channel)
            {
            case ADC_CHANNEL_CURRENT: // измерения тока
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

                current_adc_square += (digital_filter - current_offset) * (digital_filter - current_offset);
                current_adc_sum += digital_filter;
                current_adc_count++;
                break;

            case ADC_CHANNEL_SET: // задание тока
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

                setup_adc_sum += digital_filter;
                setup_adc_count++;
                break;

            default:
                err_count++;
                break;
            }
            p++;
        }

        // time2 = esp_timer_get_time();
        // ESP_LOGD("main", "time: %8lld; cnt: %d; ret: %d, %x", time2 - time1, current_adc_count, ret_num, ret);
        // continue;

        // обработка
        if (current_adc_count > 0)
        {
            int avgc = roundf(sqrtf(current_adc_square / current_adc_count));

            current_sum_rms += avgc;
            current_sum += (current_adc_sum / current_adc_count);
            avg_setup_sum += (setup_adc_sum / setup_adc_count);
            cycle_count++;

            if (cycle_count < (CYCLE / 20))
            {
                continue;
            }

            float avg_current = current_sum_rms / cycle_count;
            int avg_setup = avg_setup_sum / cycle_count;
            int offset = current_sum / cycle_count;

            cycle_count = 0;
            current_sum_rms = 0;
            current_sum = 0;
            avg_setup_sum = 0;

            if (current_offset == 0)
            {
                run_stage = 1;
                current_offset = offset;
                ESP_LOGW("main", "ADC current offset: %d", current_offset);
                continue;
            }

            if (parameters_changed != 0)
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
                k_display = get_menu_val_by_id("Kdispl");
                Isetmin = get_menu_val_by_id("Isetmin");
                Isetmax = get_menu_val_by_id("Isetmax");
                // Iporog = get_menu_val_by_id("Iporog");
                ADCmax = get_menu_val_by_id("ADCmax");
                Imin = get_menu_val_by_id("Imin");
                Imax = get_menu_val_by_id("Imax");
                Iconst = get_menu_val_by_id("Iconst");
                /*
                                if (parameters_changed == -1)
                                {
                                    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));

                                    if (xHandleWifi)
                                        xTaskNotifyGive(xHandleWifi); // включаем WiFi

                                    displ_data.curr = 0;
                                    displ_data.dots = false;
                                    displ_data.set = 0;

                                    xQueueSend(xQueueDisplay, &displ_data, 100);
                                }
                */
                parameters_changed = 0;
            }

            float current = avg_current * k_calc; // in A
            currents[currents_cnt] = (current * 1000.0f);

            float setup_current = (Isetmin + avg_setup / ADCmax * (Isetmax - Isetmin));
            if (setup_current < Isetmin + Iconst)
                setup_current = Isetmin + Iconst;

            if (setup_current > Isetmax)
                setup_current = Isetmax;

            input_error = setup_current - current;

            if (run_stage == 5)
            {
                int xx = current_xx + Iconst * 1000.0f;
                if (currents[currents_cnt] < xx &&
                    currents[(uint8_t)(currents_cnt - 1)] < xx &&
                    currents[(uint8_t)(currents_cnt - 2)] < xx &&
                    currents[(uint8_t)(currents_cnt - 3)] < xx &&
                    currents[(uint8_t)(currents_cnt - 4)] < xx)
                {
                    run_stage = 4;
                }
            }

            if (run_stage == 4) // Хол Ход
            {
                int xx = current_xx + Iconst * 1000.0f;
                if (currents[currents_cnt] > xx) // врезка
                {
                    run_stage = 5;
                    pid_handle->integral_err = (avg_setup * UINT8_MAX / ADCmax - input_error * pid_handle->Kp - input_error * pid_handle->Kd) / pid_handle->Ki;
                }

                int xxm = (Imin - Iconst) * 1000.0f;
                if (currents[currents_cnt] < xxm && currents[(uint8_t)(currents_cnt - 1)] < xxm && currents[(uint8_t)(currents_cnt - 2)] < xxm) // ВЫКЛ
                {
                    run_stage = 1;
                }

                // ESP_LOGI("I", "%i - %i",xx,xxm);
            }

            // подача вперед
            if (gpio_get_level(FEED_FORWARD_PIN) == 0)
            {
                if (sw1 <= 0)
                {
                    sw1 = 1;

                    if (gpio_get_level(DISABLE_PIN) == 0)
                        run_stage = 3;
                }
            }
            else
            {
                if (sw1 > 0)
                    sw1 = -1;
            }

            if (run_stage == 3) // фиксируем ХХ
            {
                current_xx = (currents[currents_cnt] + currents[(uint8_t)(currents_cnt - 1)]) / 2;
                ESP_LOGI("main", "XX: %d", current_xx);
                Isetmin = (float)current_xx / 1000.0f;
                // Iporog = Isetmin + Iconst + 1.0f;
                run_stage = 4;
            }

            if (run_stage == 2) // ждем стабилизации тока +-1A
            {
                int offs = Iconst * 1000.0f;
                if (INRANGE(currents[currents_cnt], currents[(uint8_t)(currents_cnt - 1)] - offs, currents[(uint8_t)(currents_cnt - 1)] + offs) &&
                    INRANGE(currents[currents_cnt], currents[(uint8_t)(currents_cnt - 2)] - offs, currents[(uint8_t)(currents_cnt - 1)] + offs) &&
                    INRANGE(currents[currents_cnt], currents[(uint8_t)(currents_cnt - 3)] - offs, currents[(uint8_t)(currents_cnt - 1)] + offs) &&
                    INRANGE(currents[currents_cnt], currents[(uint8_t)(currents_cnt - 4)] - offs, currents[(uint8_t)(currents_cnt - 1)] + offs) &&
                    INRANGE(currents[currents_cnt], currents[(uint8_t)(currents_cnt - 5)] - offs, currents[(uint8_t)(currents_cnt - 1)] + offs) &&
                    INRANGE(current, Imin, Imax))
                {
                    run_stage = 3;
                }
            }

            if (run_stage == 1) // ловим пуск пилы
            {
                int px = (Imax + Iconst) * 1000.0f;
                if (currents[currents_cnt] > px && currents[(uint8_t)(currents_cnt - 1)] > px)
                    run_stage = 2;

                // ESP_LOGD("main", "stage: %i Current: %i, %i, %i", run_stage, currents[currents_cnt], currents[(uint8_t)(currents_cnt - 1)], currents[(uint8_t)(currents_cnt - 2)]);
            };

            if (run_stage < 100)
            {
                if (gpio_get_level(DISABLE_PIN) == 1) // блокировка работы алгоритма
                    run_stage = 1;

                displ_data.curr = current;
                displ_data.dots = true;
                displ_data.set = setup_current;
            }
            else // DEBUG
            {
                displ_data.curr = avg_setup / 100;
                displ_data.dots = false;
                displ_data.set = avg_setup % 100;
            }

            // ESP_LOGD("main", "Current: %4.1f A  %4.1f * %d", current, avg_current, k_calc);

            xQueueSend(xQueueDisplay, &displ_data, 0);

            uint8_t dac1 = 0;
            uint8_t dac2 = 0;
            static float ret_result = 0;
            // float intg = pid_handle->integral_err;
            pid_handle->max_output = avg_setup * UINT8_MAX / ADCmax + (UINT8_MAX * 20 / 100);
            pid_handle->min_output = avg_setup * UINT8_MAX / ADCmax / 5;
            ESP_ERROR_CHECK(pid_compute(pid_handle, input_error, &ret_result));

            dac1 = (int)ret_result;

            if ((int)ret_result > UINT8_MAX)
            {
                dac1 = UINT8_MAX;
            }

            if (run_stage != 5)
            {
                if (avg_setup > ADCmax)
                    dac1 = UINT8_MAX;
                else
                    dac1 = avg_setup * UINT8_MAX / ADCmax;
                pid_reset_ctrl_block(pid_handle);
            }

            if (k_display * setup_current > UINT8_MAX)
                dac2 = UINT8_MAX;
            else
                dac2 = k_display * setup_current;

            // для отладки
            if (run_stage == 100)
            {
                dac1 = UINT8_MAX;
                dac2 = UINT8_MAX;
            }
            else if (run_stage == 101)
            {
                dac1 = UINT8_MAX / 2;
                dac2 = UINT8_MAX / 2;
            }
            else if (run_stage == 102)
            {
                dac1 = 0;
                dac2 = 0;
            }
            else if (run_stage == 110)
            {
                current = 10;
                if (k_display * current > UINT8_MAX)
                    dac2 = UINT8_MAX;
                else
                    dac2 = k_display * current;
            }
            else if (run_stage == 111)
            {
                current = 50;
                if (k_display * current > UINT8_MAX)
                    dac2 = UINT8_MAX;
                else
                    dac2 = k_display * current;
            }
            else if (run_stage == 999)
            {
                adc_continuous_stop(adc_handle);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                adc_continuous_start(adc_handle);
                adc_ll_digi_set_convert_limit_num(2);

                run_stage = 1;
            };

            dac_oneshot_output_voltage(chan1_handle, dac1);
            dac_oneshot_output_voltage(chan2_handle, dac2);

            time2 = esp_timer_get_time();

            ESP_LOGI("main", "%d stage: %d %8lld Current: %4.1f A (set: %.1f) Setup ADC: %4d; DAC: %4d, %.0f + %.0f + %.0f", gpio_get_level(GPIO_NUM_13), run_stage, time2 - time1, current, setup_current, avg_setup, dac1, input_error * pid_handle->Kp, pid_handle->integral_err * pid_handle->Ki, (input_error - pid_handle->previous_err2) * pid_handle->Kd);
            // printf(">PV:%.1f\n>SP:%.1f\n>MV:%d\n", current, setup_current, dac1);

            time1 = time2;
            currents_cnt++;
        }
    }
}

void displ_task(void *arg)
{
    int display_type = 0;

    ESP_ERROR_CHECK(i2cdev_init());

    i2c_dev_t i2cdev;

    uint16_t ht16data[4];

    uint16_t digits_ht16[10] = {
        0x003F, // 0
        0x0006, // 1
        0x00DB, // 2
        0x00CF, // 3
        0x00E6, // 4
        0x00ED, // 5
        0x00FD, // 6
        0x0007, // 7
        0x00FF, // 8
        0x00EF  // 9
    };

    uint16_t symbols_ht16[] = {
        0,
        0x1201, // Т
        0x04FD, // В
        0x0039, // С
        0x00F9, // Е
        0x0C86, // А
        0x00FC, // Ь
        0x0536, // М
        0x2d00, // X
        0x0d00, // Y
        0x4C36, // N.
        0x00C0, // --
        0x0037, // П
        0x0C36, // И
        0x0C06, // Л
        0x003F, // О
        0x00F3, // Р
        0x2470, // K
    };

    esp_err_t ret;

    if (display_type == 0)
    {
        ret = ht16k33_init_desc(&i2cdev, 0, GPIO_NUM_21, GPIO_NUM_22, HT16K33_DEFAULT_ADDR) | ht16k33_init(&i2cdev);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize ht16k33: %s", esp_err_to_name(ret));
        }
        else
        {
            display_type = 2;
        }
    }

    // Configure the display
    tm1637_config_t tm1637config = {
        .clk_pin = TM1637_CLK_PIN,
        .dio_pin = TM1637_DIO_PIN,
        .bit_delay_us = 100 // Default timing
    };

    // Initialize the display
    tm1637_handle_t tm1637display;

    if (display_type == 0)
    {
        ESP_ERROR_CHECK(i2cdev_done());

        ret = tm1637_init(&tm1637config, &tm1637display);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize TM1637: %s", esp_err_to_name(ret));
        }
        else
        {
            display_type = 1;
        }
    }

    ESP_LOGI(TAG, "Display initialized successfully");

    if (display_type == 1)
    {
        // Set brightness to medium (0-7", .izm = "", .val range)
        tm1637_set_brightness(tm1637display, 4, true);
    }

    if (display_type == 2)
    {
        ESP_ERROR_CHECK(ht16k33_display_setup(&i2cdev, 1, HTK16K33_F_0HZ));
        /*
                for (int i = 0; i < 10; i++)
                {
                    ht16data[0] = digits_ht16[i];
                    ht16data[1] = 0;
                    ht16data[2] = 0;
                    ht16data[3] = 0;
                    ESP_ERROR_CHECK(ht16k33_ram_write(&i2cdev, (uint8_t *)ht16data));
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
        */

        // пилорамка
        const int hello[17] = {0, 0, 0, 0, 12, 13, 14, 15, 16, 5, 7, 17, 5, 0, 0, 0, 0};
        for (int i = 0; i < 14; i++)
        {
            ht16data[0] = symbols_ht16[hello[i]];
            ht16data[1] = symbols_ht16[hello[i + 1]];
            ht16data[2] = symbols_ht16[hello[i + 2]];
            ht16data[3] = symbols_ht16[hello[i + 3]];
            ESP_ERROR_CHECK(ht16k33_ram_write(&i2cdev, (uint8_t *)ht16data));
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }

    displ_t displ_data;
    uint8_t tm1637char_display[TM1637_MAX_DIGITS];

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

            if (display_type == 1) // tm1637
            {
                tm1637char_display[0] = (val1 >= 10.0) ? tm1637_encode_digit(val1 / 10) : 0;
                tm1637char_display[1] = tm1637_encode_digit(val1 % 10) | ((displ_data.dots == true) ? TM1637_SEG_DP : 0);
                tm1637char_display[2] = tm1637_encode_digit(val2 / 10);
                tm1637char_display[3] = tm1637_encode_digit(val2 % 10);

                tm1637_set_segments(tm1637display, tm1637char_display, 4, 0);
            }

            if (display_type == 2) // VK16K33
            {
                ht16data[0] = digits_ht16[val1 / 10];
                ht16data[1] = digits_ht16[val1 % 10] | ((displ_data.dots == true) ? 0x4000 : 0); //+ dot
                ht16data[2] = digits_ht16[val2 / 10];
                ht16data[3] = digits_ht16[val2 % 10];
                ESP_ERROR_CHECK(ht16k33_ram_write(&i2cdev, (uint8_t *)ht16data));
            }
        }
        vTaskDelay(1);
    }
}