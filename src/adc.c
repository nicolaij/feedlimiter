#include "main.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "esp_err.h"
#include "esp_partition.h"
#include "esp_system.h"

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
#define ADC_READ_TIMEOUT_MS 1000

extern QueueHandle_t xQueueDisplay;

adc_continuous_handle_t adc_handle = NULL;

volatile int run_stage = 1;
portMUX_TYPE g_control_mux = portMUX_INITIALIZER_UNLOCKED;

static const char *TAG = "adc";

#define MEDIAN(a) (((MAX(a[0], a[1]) == MAX(a[1], a[2])) ? MAX(a[0], a[2]) : MAX(a[1], MIN(a[0], a[2]))))
#define INRANGE(value, min, max) (((value) < (max)) && ((value) > (min)))

#if !defined DISPLAY_ONLY

dac_oneshot_handle_t feed_dac_handle;
dac_oneshot_handle_t display_dac_handle;

static uint8_t dac_clamp(float dac_value)
{
    if (!isfinite(dac_value) || dac_value <= 0.0f)
        return 0;
    if (dac_value >= UINT8_MAX)
        return UINT8_MAX;
    return (uint8_t)(dac_value + 0.5f);
}

static void adc_restart(uint32_t delay_ms)
{
    dac_oneshot_output_voltage(feed_dac_handle, 0);
    dac_oneshot_output_voltage(display_dac_handle, 0);

    esp_err_t err = adc_continuous_stop(adc_handle);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_LOGW(TAG, "ADC stop: %s", esp_err_to_name(err));

    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    err = adc_continuous_start(adc_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC restart failed: %s", esp_err_to_name(err));
        esp_restart();
    }

    run_stage = 1;
}

static void load_pid_params(float *kp, float *ki, float *kd)
{
    *kp = get_menu_val_by_id("pidP");
    *ki = get_menu_val_by_id("pidI");
    *kd = get_menu_val_by_id("pidD");
}

static void adc_continuous_init()
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = POINTS10 * 2 * SOC_ADC_DIGI_RESULT_BYTES * 2,
        .conv_frame_size = POINTS10 * 2 * SOC_ADC_DIGI_RESULT_BYTES,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t adc_digi_config = {
        .sample_freq_hz = 20000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    adc_digi_config.pattern_num = 2;

    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = ADC_CHANNEL_CURRENT;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_pattern[1].atten = ADC_ATTEN_DB_12;
    adc_pattern[1].channel = ADC_CHANNEL_SET;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_digi_config.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_digi_config));

    dac_oneshot_config_t feed_dac_config = {
        .chan_id = DAC_CHAN_0,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&feed_dac_config, &feed_dac_handle));

    dac_oneshot_config_t display_dac_config = {
        .chan_id = DAC_CHAN_1,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&display_dac_config, &display_dac_handle));
}

void adc_task(void *arg)
{
    esp_err_t err;
    uint32_t bytes_read = 0;
    uint8_t adc_buffer[POINTS10 * 2 * SOC_ADC_DIGI_RESULT_BYTES] = {0};
    int current_median_samples[3];
    uint8_t current_median_count = 0;
    int setpoint_median_samples[3];
    uint8_t setpoint_median_count = 0;
    int filtered_sample;

    int adc_zero_offset = 0;
    bool zero_offset_calibrated = false;

    int cycles_in_window = 0;
    int current_rms_sum = 0;
    int current_raw_sum = 0;

    int setpoint_raw_sum = 0;

    int current_history_ma[UINT8_MAX + 1] = {0}; // in mA
    uint8_t history_index = 0;
    uint8_t history_count = 0;

    int idle_current_ma = 0; // in mA

    int feed_forward_state = 0;

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gpio_config_t io_config = {};
    io_config.intr_type = GPIO_INTR_DISABLE;
    io_config.pin_bit_mask = BIT64(FEED_FORWARD_PIN) | BIT64(DISABLE_PIN);
    // set as input mode
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_config));

    float adc_to_amps = get_menu_val_by_id("Kcalc");
    float amps_to_dac = get_menu_val_by_id("Kdispl");
    float target_current_min_a = get_menu_val_by_id("Isetmin");
    float target_current_max_a = get_menu_val_by_id("Isetmax");
    // float Iporog = get_menu_val_by_id("Iporog");
    float setpoint_adc_max = get_menu_val_by_id("ADCmax");
    float setpoint_curve_exponent = get_menu_val_by_id("ADCkLin"); // степень 0..1..10000

    adc_zero_offset = get_menu_val_by_id("offsetADC");
    zero_offset_calibrated = (adc_zero_offset != 0);

    float idle_current_min_a = get_menu_val_by_id("Imin");
    float idle_current_max_a = get_menu_val_by_id("Imax");
    float idle_current_tolerance_a = get_menu_val_by_id("Iconst");

    float position_min_pct = get_menu_val_by_id("Ipolmin");
    float position_max_pct = get_menu_val_by_id("Ipolmax");

    // Увелич. скорости при работе ПИД
    float pid_overspeed_pct = get_menu_val_by_id("OVRSpeed");

    // DAC.Минимальная скорость пиления
    float min_feed_dac = get_menu_val_by_id("DACMinOut");

    displ_t display_data;

    pid_ctrl_block_handle_t pid;

    float pid_kp;
    float pid_ki;
    float pid_kd;
    load_pid_params(&pid_kp, &pid_ki, &pid_kd);

    pid_ctrl_config_t pid_config = {.init_param.cal_type = PID_CAL_TYPE_POSITIONAL,
                                  .init_param.kp = pid_kp,
                                  .init_param.ki = pid_ki,
                                  .init_param.kd = pid_kd,
                                  .init_param.max_integral = 255,
                                  .init_param.min_integral = 0,
                                  .init_param.max_output = 255,
                                  .init_param.min_output = 0};

    adc_continuous_init();

    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid));

    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    // adc_ll_digi_set_convert_limit_num(2);

    int64_t cycle_start_us = esp_timer_get_time();
    int64_t cycle_end_us = esp_timer_get_time();

    float current_error_a = 0;
    float feed_speed = 0;

    while (1)
    {
        // time1 = esp_timer_get_time();

        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        adc_digi_output_data_t *adc_sample = (void *)adc_buffer;
        int64_t current_square_sum = 0;
        int current_sample_sum = 0;
        int current_sample_count = 0;
        int setpoint_sample_sum = 0;
        int setpoint_sample_count = 0;

        err = adc_continuous_read(adc_handle, adc_buffer, POINTS10 * 2 * SOC_ADC_DIGI_RESULT_BYTES, &bytes_read, ADC_READ_TIMEOUT_MS);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "ADC read failed (%s), restarting", esp_err_to_name(err));
            memset(current_history_ma, 0, sizeof(current_history_ma));
            history_index = 0;
            history_count = 0;
            current_median_count = 0;
            setpoint_median_count = 0;
            cycles_in_window = 0;
            current_rms_sum = 0;
            current_raw_sum = 0;
            setpoint_raw_sum = 0;
            adc_restart(100);
            continue;
        }

        while ((uint8_t *)adc_sample < adc_buffer + bytes_read)
        {
            switch (adc_sample->type1.channel)
            {
            case ADC_CHANNEL_CURRENT: // измерения тока
                current_median_samples[current_median_count % 3] = adc_sample->type1.data;
                current_median_count++;

                if (current_median_count >= 3)
                {
                    filtered_sample = MEDIAN(current_median_samples);
                    if (current_median_count >= 6)
                        current_median_count = 3;
                }
                else
                {
                    filtered_sample = adc_sample->type1.data;
                }

                int64_t delta = filtered_sample - adc_zero_offset;
                current_square_sum += delta * delta;
                current_sample_sum += filtered_sample;
                current_sample_count++;
                break;

            case ADC_CHANNEL_SET: // задание тока
                setpoint_median_samples[setpoint_median_count % 3] = adc_sample->type1.data;
                setpoint_median_count++;

                if (setpoint_median_count >= 3)
                {
                    filtered_sample = MEDIAN(setpoint_median_samples);
                    if (setpoint_median_count >= 6)
                        setpoint_median_count = 3;
                }
                else
                {
                    filtered_sample = adc_sample->type1.data;
                }

                setpoint_sample_sum += filtered_sample;
                setpoint_sample_count++;
                break;

            default:
                break;
            }
            adc_sample++;
        }

        // time2 = esp_timer_get_time();
        // ESP_LOGD("main", "time: %8lld; cnt: %d; ret: %d, %x", time2 - time1, current_adc_count, ret_num, ret);
        // continue;

        // обработка
        if (current_sample_count > 0)
        {
            if (setpoint_sample_count == 0)
            {
                continue;
            }

            int rms_sample = roundf(sqrtf((float)current_square_sum / current_sample_count));

            current_rms_sum += rms_sample;
            current_raw_sum += (current_sample_sum / current_sample_count);
            setpoint_raw_sum += (setpoint_sample_sum / setpoint_sample_count);
            cycles_in_window++;

            if (cycles_in_window < (CYCLE / 20))
            {
                continue;
            }

            float average_rms_adc = current_rms_sum / cycles_in_window;
            int average_setpoint_adc = setpoint_raw_sum / cycles_in_window;
            int measured_zero_offset = current_raw_sum / cycles_in_window;

            cycles_in_window = 0;
            current_rms_sum = 0;
            current_raw_sum = 0;
            setpoint_raw_sum = 0;

            if (!zero_offset_calibrated)
            {
                run_stage = 1;
                adc_zero_offset = measured_zero_offset;
                zero_offset_calibrated = true;
                ESP_LOGW("main", "ADC current offset: %d", adc_zero_offset);
                continue;
            }

            int pending_changed;
            taskENTER_CRITICAL(&g_control_mux);
            pending_changed = parameters_changed;
            parameters_changed = 0;
            taskEXIT_CRITICAL(&g_control_mux);

            if (pending_changed != 0)
            {
                float pid_kp;
                float pid_ki;
                float pid_kd;
                load_pid_params(&pid_kp, &pid_ki, &pid_kd);

                pid_ctrl_parameter_t pid_parameters = {.cal_type = PID_CAL_TYPE_POSITIONAL,
                                               .kp = pid_kp,
                                               .ki = pid_ki,
                                               .kd = pid_kd,
                                               .max_integral = 255,
                                               .min_integral = 0,
                                               .max_output = 255,
                                               .min_output = 0};

                pid_update_parameters(pid, &pid_parameters);

                adc_to_amps = get_menu_val_by_id("Kcalc");
                amps_to_dac = get_menu_val_by_id("Kdispl");
                target_current_min_a = get_menu_val_by_id("Isetmin");
                target_current_max_a = get_menu_val_by_id("Isetmax");
                position_min_pct = get_menu_val_by_id("Ipolmin");
                position_max_pct = get_menu_val_by_id("Ipolmax");
                // Iporog = get_menu_val_by_id("Iporog");
                setpoint_adc_max = get_menu_val_by_id("ADCmax");
                idle_current_min_a = get_menu_val_by_id("Imin");
                idle_current_max_a = get_menu_val_by_id("Imax");
                idle_current_tolerance_a = get_menu_val_by_id("Iconst");
                setpoint_curve_exponent = get_menu_val_by_id("ADCkLin"); // степень 0..1..10000

                pid_overspeed_pct = get_menu_val_by_id("OVRSpeed");
                min_feed_dac = get_menu_val_by_id("DACMinOut");

                adc_zero_offset = get_menu_val_by_id("offsetADC");
                zero_offset_calibrated = (adc_zero_offset != 0);

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
            }

            // Текущий ток в А
            float current_a = average_rms_adc * adc_to_amps; // in A
            if (!isfinite(current_a))
                current_a = 0.0f;

            float current_ma = current_a * 1000.0f;
            if (!isfinite(current_ma))
                current_ma = 0.0f;
            if (current_ma > INT_MAX)
                current_ma = INT_MAX;
            else if (current_ma < INT_MIN)
                current_ma = INT_MIN;
            current_history_ma[history_index] = (int)current_ma;

            if (history_count < 6)
                history_count++;
            bool history_ready = (history_count >= 6);

            // float setup_current = (Isetmin + avg_setup / ADCmax * (Isetmax - Isetmin));
            // float ADCk2 = ((Isetmax - Isetmin) - (ADCk1 * ADCmax)) / (ADCmax * ADCmax);
            // float setup_current = Isetmin + ADCk1 * avg_setup + ADCk2 * avg_setup * avg_setup;
            // float setup_current = (Isetmin + avg_setup * (Isetmax - Isetmin) / ADCmax); // линейно

            // Линейное положение регулятора 0..100%
            float setpoint_position_pct = average_setpoint_adc * 100.0f / setpoint_adc_max;
            if (!isfinite(setpoint_position_pct))
                setpoint_position_pct = 0.0f;
            else if (setpoint_position_pct < 0.0f)
                setpoint_position_pct = 0.0f;
            else if (setpoint_position_pct > 100.0f)
                setpoint_position_pct = 100.0f;

            // задание тока ~10..30А (Isetmin..Isetmax)
            float target_current_a = target_current_min_a;
            float position_span_pct = position_max_pct - position_min_pct;
            float target_current_span_a = target_current_max_a - target_current_min_a;
            if (isfinite(position_span_pct) && position_span_pct > 0.0f &&
                isfinite(target_current_span_a) && target_current_span_a > 0.0f &&
                isfinite(setpoint_curve_exponent) && setpoint_curve_exponent > 0.0f)
            {
                float position_ratio = (setpoint_position_pct - position_min_pct) / position_span_pct;
                if (position_ratio < 0.0f)
                    position_ratio = 0.0f;
                else if (position_ratio > 1.0f)
                    position_ratio = 1.0f;

                float curve_ratio = powf(position_ratio, setpoint_curve_exponent);
                if (isfinite(curve_ratio))
                    target_current_a = target_current_min_a + curve_ratio * target_current_span_a;
            }

            if (!isfinite(target_current_a))
                target_current_a = target_current_min_a;
            else if (target_current_a < target_current_min_a)
                target_current_a = target_current_min_a;
            else if (target_current_a > target_current_max_a)
                target_current_a = target_current_max_a;

            /*
                        float Isetminlimit = 0;
                        //(float)current_xx / 1000.0f;
                        if (Isetmin > Isetminlimit)
                            Isetminlimit = Isetmin;

                        if (setup_current < Isetminlimit)
                            setup_current = Isetminlimit;

                        if (setup_current > Isetmax)
                            setup_current = Isetmax;
            */
            current_error_a = target_current_a - current_a;

            if (run_stage == 5 && history_ready) // пилим
            {
                int idle_threshold_ma = idle_current_ma + idle_current_tolerance_a * 1000.0f;
                if (current_history_ma[history_index] < idle_threshold_ma &&
                    current_history_ma[(uint8_t)(history_index - 1)] < idle_threshold_ma &&
                    current_history_ma[(uint8_t)(history_index - 2)] < idle_threshold_ma &&
                    current_history_ma[(uint8_t)(history_index - 3)] < idle_threshold_ma &&
                    current_history_ma[(uint8_t)(history_index - 4)] < idle_threshold_ma)
                {
                    run_stage = 4;
                }
            }

            if (run_stage == 4 && history_ready) // Хол Ход
            {
                int idle_threshold_ma = idle_current_ma + idle_current_tolerance_a * 1000.0f;
                if (current_history_ma[history_index] > idle_threshold_ma) // врезка
                {
                    run_stage = 5;
                    if (isfinite(pid->Ki) && pid->Ki > 0.000001f)
                        pid->integral_err = (feed_speed - current_error_a * pid->Kp - current_error_a * pid->Kd) / pid->Ki;
                    else
                        pid->integral_err = 0.0f;
                    // pid_handle->integral_err = (avg_setup * UINT8_MAX / ADCmax - input_error * pid_handle->Kp - input_error * pid_handle->Kd) / pid_handle->Ki;
                    // pid_handle->integral_err = ((setup_current - Isetminlimit) * UINT8_MAX / (Isetmax - Isetmin) - input_error * pid_handle->Kp - input_error * pid_handle->Kd) / pid_handle->Ki;
                }

                int stop_threshold_ma = (idle_current_min_a - idle_current_tolerance_a) * 1000.0f;
                if (current_history_ma[history_index] < stop_threshold_ma && current_history_ma[(uint8_t)(history_index - 1)] < stop_threshold_ma && current_history_ma[(uint8_t)(history_index - 2)] < stop_threshold_ma) // ВЫКЛ
                {
                    run_stage = 1;
                }

                // ESP_LOGI("I", "%i - %i",xx,xxm);
            }

            // подача вперед
            if (gpio_get_level(FEED_FORWARD_PIN) == 0)
            {
                if (feed_forward_state <= 0)
                {
                    feed_forward_state = 1;

                    if (gpio_get_level(DISABLE_PIN) == 0 && history_ready)
                        run_stage = 3;
                }
            }
            else
            {
                if (feed_forward_state > 0)
                    feed_forward_state = -1;

                if (run_stage == 5)
                    run_stage = 4;
            }

            if (run_stage == 3 && history_ready) // фиксируем ХХ
            {
                idle_current_ma = (current_history_ma[history_index] + current_history_ma[(uint8_t)(history_index - 1)]) / 2;
                ESP_LOGI("main", "XX: %d", idle_current_ma);
                // Isetmin = (float)current_xx / 1000.0f;
                // Iporog = Isetmin + Iconst + 1.0f;
                run_stage = 4;
            }

            if (run_stage == 2 && history_ready) // ждем стабилизации тока +-1A
            {
                int tolerance_ma = idle_current_tolerance_a * 1000.0f;
                if (INRANGE(current_history_ma[history_index], current_history_ma[(uint8_t)(history_index - 1)] - tolerance_ma, current_history_ma[(uint8_t)(history_index - 1)] + tolerance_ma) &&
                    INRANGE(current_history_ma[history_index], current_history_ma[(uint8_t)(history_index - 2)] - tolerance_ma, current_history_ma[(uint8_t)(history_index - 2)] + tolerance_ma) &&
                    INRANGE(current_history_ma[history_index], current_history_ma[(uint8_t)(history_index - 3)] - tolerance_ma, current_history_ma[(uint8_t)(history_index - 3)] + tolerance_ma) &&
                    INRANGE(current_history_ma[history_index], current_history_ma[(uint8_t)(history_index - 4)] - tolerance_ma, current_history_ma[(uint8_t)(history_index - 4)] + tolerance_ma) &&
                    INRANGE(current_history_ma[history_index], current_history_ma[(uint8_t)(history_index - 5)] - tolerance_ma, current_history_ma[(uint8_t)(history_index - 5)] + tolerance_ma) &&
                    INRANGE(current_a, idle_current_min_a, idle_current_max_a))
                {
                    run_stage = 3;
                }
            }

            if (run_stage == 1 && history_ready) // ловим пуск пилы
            {
                int start_threshold_ma = (idle_current_max_a + idle_current_tolerance_a) * 1000.0f;
                if (current_history_ma[history_index] > start_threshold_ma && current_history_ma[(uint8_t)(history_index - 1)] > start_threshold_ma)
                    run_stage = 2;

                // ESP_LOGD("main", "stage: %i Current: %i, %i, %i", run_stage, currents[currents_cnt], currents[(uint8_t)(currents_cnt - 1)], currents[(uint8_t)(currents_cnt - 2)]);
            };

            if (run_stage < 100)
            {
                if (gpio_get_level(DISABLE_PIN) == 1) // блокировка работы алгоритма
                    run_stage = 1;

                display_data.curr = (current_a > 1000.0f) ? 1000 : ((current_a < -1000.0f) ? -1000 : (int)(current_a + 0.5f)); // округление
                display_data.dots = true;
                display_data.set = target_current_a + 0.5; // округление
            }
            else // DEBUG
            {
                display_data.curr = average_setpoint_adc / 100;
                display_data.dots = false;
                display_data.set = average_setpoint_adc % 100;
            }

            // ESP_LOGD("main", "Current: %4.1f A %d", current, displ_data.curr);

            xQueueSend(xQueueDisplay, &display_data, 0);

            uint8_t feed_dac = 0;
            uint8_t display_dac = 0;
            static float pid_output = 0;
            // float intg = pid_handle->integral_err;

            feed_speed = setpoint_position_pct * UINT8_MAX / 100.0f;
            if (!isfinite(feed_speed))
                feed_speed = 0.0f;
            else if (feed_speed < 0.0f)
                feed_speed = 0.0f;
            else if (feed_speed > UINT8_MAX)
                feed_speed = UINT8_MAX;

            float pid_max_output = feed_speed + (UINT8_MAX * pid_overspeed_pct / 100.0f);
            if (!isfinite(pid_max_output) || pid_max_output > UINT8_MAX)
                pid_max_output = UINT8_MAX;
            pid->max_output = pid_max_output;
            // pid_handle->min_output = setup_speed / 10;
            // if (pid_handle->min_output < DACMinOut)
            pid->min_output = min_feed_dac;
            pid->max_integral = pid->max_output;
            ESP_ERROR_CHECK(pid_compute(pid, current_error_a, &pid_output));

            if (run_stage == 5) // автоматический режим работы
            {
                feed_dac = dac_clamp(pid_output);
            }
            else
            {
                feed_dac = dac_clamp(feed_speed);

                pid_reset_ctrl_block(pid);
            }

            display_dac = dac_clamp(amps_to_dac * target_current_a);

            // для отладки
            if (run_stage == 100)
            {
                feed_dac = UINT8_MAX;
                display_dac = UINT8_MAX;
            }
            else if (run_stage == 101)
            {
                feed_dac = UINT8_MAX / 2;
                display_dac = UINT8_MAX / 2;
            }
            else if (run_stage == 102)
            {
                feed_dac = 0;
                display_dac = 0;
            }
            else if (run_stage == 110)
            {
                current_a = 10;
                display_dac = dac_clamp(amps_to_dac * current_a);
            }
            else if (run_stage == 111)
            {
                current_a = 50;
                display_dac = dac_clamp(amps_to_dac * current_a);
            }
            else if (run_stage == 999)
            {
                memset(current_history_ma, 0, sizeof(current_history_ma));
                history_index = 0;
                history_count = 0;
                current_median_count = 0;
                setpoint_median_count = 0;
                cycles_in_window = 0;
                current_rms_sum = 0;
                current_raw_sum = 0;
                setpoint_raw_sum = 0;
                adc_restart(1500);
                continue;
            };

            dac_oneshot_output_voltage(feed_dac_handle, feed_dac);
            dac_oneshot_output_voltage(display_dac_handle, display_dac);

            cycle_end_us = esp_timer_get_time();

            char log_line[256];
            int log_length = snprintf(log_line, sizeof(log_line), "%d stage: %d %8lld Current: %4.1f A (set: %.1f, ADC: %4d); DAC: %3d, %.0f + %.0f + %.0f", gpio_get_level(FEED_FORWARD_PIN), run_stage, cycle_end_us - cycle_start_us, current_a, target_current_a, average_setpoint_adc, feed_dac, current_error_a * pid->Kp, pid->integral_err * pid->Ki, (current_error_a - pid->previous_err2) * pid->Kd);
            ESP_LOGI("main", "%s", log_line);
            // printf(">PV:%.1f\n>SP:%.1f\n>MV:%d\n", current, setup_current, dac1);

            if (log_length > 0 && log_length < (int)sizeof(log_line))
                ws_send_text(log_line, log_length);

            cycle_start_us = cycle_end_us;
            history_index++;
        }
    }
}

#endif

#if defined DISPLAY_ONLY
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *payload, int payload_len)
{
    uint8_t *mac_address = recv_info->src_addr;
    uint8_t *destination_mac = recv_info->des_addr;

    if (mac_address == NULL || payload == NULL || destination_mac == NULL || payload_len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (payload_len != sizeof(displ_t))
    {
        ESP_LOGW(TAG, "Unexpected ESPNOW payload: %d", payload_len);
        return;
    }

    displ_t display_message;
    memcpy(&display_message, payload, sizeof(display_message));
    xQueueSend(xQueueDisplay, &display_message, 0);

    if (IS_BROADCAST_ADDR(destination_mac))
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data: %d, %d", display_message.curr, display_message.set);
    else
        ESP_LOGD(TAG, "Receive unicast ESPNOW data: %d, %d", display_message.curr, display_message.set);
}
#endif

void display_task(void *arg)
{

    int display_kind = 0;

    ESP_ERROR_CHECK(i2cdev_init());

    i2c_dev_t i2c_dev;

    uint16_t ht16_frame[4];

    uint16_t ht16_digits[10] = {
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

    uint16_t ht16_symbols[] = {
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

    esp_err_t err;

    if (display_kind == 0)
    {
        err = ht16k33_init_desc(&i2c_dev, 0, SDA_PIN, SCL_PIN, HT16K33_DEFAULT_ADDR);
        if (err == ESP_OK)
            err = ht16k33_init(&i2c_dev);
        if (err != ESP_OK)
        {
            ESP_LOGE("dislay", "Failed to initialize ht16k33: %s", esp_err_to_name(err));
        }
        else
        {
            display_kind = 2;
        }
    }

    // Configure the display
    tm1637_config_t tm1637_cfg = {
        .clk_pin = SCL_PIN,
        .dio_pin = SDA_PIN,
        .bit_delay_us = 100 // Default timing
    };

    // Initialize the display
    tm1637_handle_t tm1637_display;
    uint8_t tm1637_segments[TM1637_MAX_DIGITS];

    if (display_kind == 0)
    {
        ESP_ERROR_CHECK(i2cdev_done());

        err = tm1637_init(&tm1637_cfg, &tm1637_display);
        if (err != ESP_OK)
        {
            ESP_LOGE("dislay", "Failed to initialize TM1637: %s", esp_err_to_name(err));
        }
        else
        {
            display_kind = 1;
        }
    }

    ESP_LOGI("dislay", "Display initialized successfully");

    if (display_kind == 1)
    {
        // Set brightness to medium (0-7", .izm = "", .val range)
        tm1637_set_brightness(tm1637_display, 7, true);
        vTaskDelay(pdMS_TO_TICKS(250));

        tm1637_scroll_text(tm1637_display, "HELLO    ", 200);
    }

    if (display_kind == 2)
    {
        ESP_ERROR_CHECK(ht16k33_display_setup(&i2c_dev, 1, HTK16K33_F_0HZ));
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
        const int saw_symbols[17] = {0, 0, 0, 0, 12, 13, 14, 15, 16, 5, 7, 17, 5, 0, 0, 0, 0};
        for (int frame_index = 0; frame_index < 14; frame_index++)
        {
            ht16_frame[0] = ht16_symbols[saw_symbols[frame_index]];
            ht16_frame[1] = ht16_symbols[saw_symbols[frame_index + 1]];
            ht16_frame[2] = ht16_symbols[saw_symbols[frame_index + 2]];
            ht16_frame[3] = ht16_symbols[saw_symbols[frame_index + 3]];
            ESP_ERROR_CHECK(ht16k33_ram_write(&i2c_dev, (uint8_t *)ht16_frame));
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }

    displ_t display_data;

    bool espnow_enabled = true;

#if !defined DISPLAY_ONLY
    uint8_t mac_address[6];
    int mac_high = get_menu_val_by_id("MAC1");
    int mac_low = get_menu_val_by_id("MAC2");
    mac_address[0] = (mac_high >> 16) & 0xFF;
    mac_address[1] = (mac_high >> 8) & 0xFF;
    mac_address[2] = (mac_high >> 0) & 0xFF;
    mac_address[3] = (mac_low >> 16) & 0xFF;
    mac_address[4] = (mac_low >> 8) & 0xFF;
    mac_address[5] = (mac_low >> 0) & 0xFF;
    espnow_enabled = (mac_high != 0 || mac_low != 0);
#endif

    if (espnow_enabled)
    {
        if (xHandleWifi)
            xTaskNotify(xHandleWifi, NOTYFY_WIFI_ESPNOW, eSetValueWithOverwrite);
        run_stage_set(999);

        // ESP-NOW requires WiFi to be started before esp_now_init().
        if (xWifiReadySemaphore)
        {
            if (xSemaphoreTake(xWifiReadySemaphore, pdMS_TO_TICKS(5000)) != pdTRUE)
                ESP_LOGW(TAG, "WiFi ready timeout, retrying ESP-NOW");
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1200));
        }

        esp_err_t espnow_err = ESP_FAIL;
        for (int attempt = 0; attempt < 10; attempt++)
        {
            espnow_err = esp_now_init();
            if (espnow_err == ESP_OK)
                break;
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (espnow_err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(espnow_err));
        }
        else
        {
            esp_now_peer_info_t peer_info = {.ifidx = WIFI_IF_AP, .peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, .channel = 0, .lmk = "", .encrypt = false};
#if defined DISPLAY_ONLY
            esp_err_t cb_err = esp_now_register_recv_cb(espnow_recv_cb);
            if (cb_err != ESP_OK)
                ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(cb_err));
#else
            memcpy(peer_info.peer_addr, mac_address, ESP_NOW_ETH_ALEN);
#endif
            esp_err_t peer_err = esp_now_add_peer(&peer_info);
            if (peer_err != ESP_OK && peer_err != ESP_ERR_ESPNOW_EXIST)
                ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(peer_err));
        }
    }

    while (1)
    {
        if (xQueueReceive(xQueueDisplay, &display_data, 3000 / portTICK_PERIOD_MS) == pdPASS)
        {
            int display_current = display_data.curr;
            if (display_current > 99)
                display_current = 99;
            if (display_current < 0)
                display_current = 0;

            int display_setpoint = display_data.set;
            if (display_setpoint > 99)
                display_setpoint = 99;
            if (display_setpoint < 0)
                display_setpoint = 0;

            if (display_kind == 1) // tm1637
            {
                tm1637_segments[0] = (display_current >= 10.0) ? tm1637_encode_digit(display_current / 10) : 0;
                tm1637_segments[1] = tm1637_encode_digit(display_current % 10) | ((display_data.dots == true) ? TM1637_SEG_DP : 0);
                tm1637_segments[2] = tm1637_encode_digit(display_setpoint / 10);
                tm1637_segments[3] = tm1637_encode_digit(display_setpoint % 10);

                tm1637_set_segments(tm1637_display, tm1637_segments, 4, 0);
            }

            if (display_kind == 2) // VK16K33
            {
                ht16_frame[0] = ht16_digits[display_current / 10];
                ht16_frame[1] = ht16_digits[display_current % 10] | ((display_data.dots == true) ? 0x4000 : 0); //+ dot
                ht16_frame[2] = ht16_digits[display_setpoint / 10];
                ht16_frame[3] = ht16_digits[display_setpoint % 10];
                ESP_ERROR_CHECK(ht16k33_ram_write(&i2c_dev, (uint8_t *)ht16_frame));
            }
#if !defined DISPLAY_ONLY
            if (mac_high != 0 || mac_low != 0)
                esp_now_send(mac_address, (uint8_t *)&display_data, sizeof(displ_t));
#endif
        }
        else
        {
            display_data.curr = 0;
            display_data.set = 0;
            display_data.dots = 0;
            xQueueSend(xQueueDisplay, &display_data, 0);
        }
        vTaskDelay(1);
    }
}