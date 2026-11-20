#include "main.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_event.h"
#include "driver/gpio.h"
#include "button_gpio.h"
#include "iot_button.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "led_indicator_gpio.h"

static const char *TAG = "terminal";

static led_indicator_handle_t status_led_handle = NULL;

extern QueueHandle_t xQueueDisplay;

volatile int parameters_changed = 0;

menu_t menu[] = {
    /*0*/ {.id = "idn", .name = "Номер устройства", .izm = "", .val = 1, .min = 1, .max = 1000000},
    // {.id = "waitwifi", .name = "Ожидание WiFi", .izm = "мин", .val = 3, .min = 1, .max = 1000000},
    {.id = "MAC1", .name = "ESPNOW! Target MAC", .izm = "", .val = 0, .min = 0, .max = 0xFFFFFF},
    {.id = "MAC2", .name = "", .izm = "", .val = 0, .min = 0, .max = 0xFFFFFF},
    {.id = "offsetADC", .name = "Смещение 0 ADC", .izm = "", .val = 1360.0, .min = 0, .max = 3000},
    {.id = "Kcalc", .name = "К масшт. ADC -> I", .izm = "", .val = 0.036, .min = 0.000001, .max = 999999},
    {.id = "Kdispl", .name = "К масшт. I -> DAC", .izm = "", .val = 5.0, .min = 0, .max = 999999},
    {.id = "Imin", .name = "Минимально возможный I ХХ дв.", .izm = "А", .val = 10.0, .min = 0, .max = 100},
    {.id = "Imax", .name = "Максимально возможный I ХХ дв.", .izm = "А", .val = 30.0, .min = 0, .max = 100},
    {.id = "Iconst", .name = "Стабильность I ХХ двигателя", .izm = "+-А", .val = 1.0, .min = 0, .max = 100},
    //{.id = "Iporog", .name = "Порог тока при врезке", .izm = "А", .val = 21.0, .min = 0, .max = 100},
    {.id = "Isetmin", .name = "Минимум I задание", .izm = "А", .val = 10.0, .min = 0, .max = 100},
    {.id = "Ipolmin", .name = "Ограничение мин. положения", .izm = "%", .val = 10.0, .min = 0, .max = 100},
    {.id = "Isetmax", .name = "Максимум I задание", .izm = "А", .val = 30.0, .min = 0, .max = 100},
    {.id = "Ipolmax", .name = "Ограничение макс. положения", .izm = "%", .val = 80.0, .min = 0, .max = 100},
    {.id = "ADCmax", .name = "Максимум ADC задание", .izm = "", .val = 3000, .min = 1, .max = 10000},
    /* ADCkLin = <1 - Быстрый старт
       ADCkLin = 1 - Линейная зависимость регулятора 
       ADCkLin = >1 - пологий старт
       */
    {.id = "ADCkLin", .name = "Степень регулятора", .izm = "", .val = 1, .min = 0.01, .max = 100000},
    {.id = "OVRSpeed", .name = "Увелич. скорости при работе ПИД", .izm = "%", .val = 10, .min = 0, .max = 100},
    {.id = "DACMinOut", .name = "DAC.Минимальная скорость пиления", .izm = "", .val = 3, .min = 0, .max = 200},
    {.id = "pidP", .name = "PID P", .izm = "", .val = 0.1000, .min = 0.000001, .max = 999999},
    {.id = "pidI", .name = "PID I", .izm = "", .val = 1.0, .min = 0.000001, .max = 999999},
    {.id = "pidD", .name = "PID D", .izm = "", .val = 0, .min = 0, .max = 999999},
    //{.id = "pidintMax", .name = "PID Intergal maximum", .izm = "", .val = 255, .min = -999999, .max = 999999},
    //{.id = "pidintMin", .name = "PID Intergal minimum", .izm = "", .val = 0, .min = -999999, .max = 999999},
    //{.id = "pidMax", .name = "PID Out maximum", .izm = "", .val = 255, .min = 0, .max = 999999},
    //{.id = "pidMin", .name = "PID Out minimum", .izm = "", .val = 0, .min = 0, .max = 999999},
};

static bool is_menu_value_valid(int menu_index, float value)
{
    if (!isfinite(value))
        return false;

    const char *id = menu[menu_index].id;

    if (strcmp(id, "Ipolmin") == 0 && value >= get_menu_val_by_id("Ipolmax"))
        return false;
    if (strcmp(id, "Ipolmax") == 0 && value <= get_menu_val_by_id("Ipolmin"))
        return false;
    if (strcmp(id, "Isetmin") == 0 && value >= get_menu_val_by_id("Isetmax"))
        return false;
    if (strcmp(id, "Isetmax") == 0 && value <= get_menu_val_by_id("Isetmin"))
        return false;
    if (strcmp(id, "Imin") == 0 && value >= get_menu_val_by_id("Imax"))
        return false;
    if (strcmp(id, "Imax") == 0 && value <= get_menu_val_by_id("Imin"))
        return false;

    return true;
}

esp_err_t init_nvs()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    // Example of nvs_get_stats() to get the number of used entries and free entries:
    nvs_stats_t nvs_stats;
    nvs_get_stats(NULL, &nvs_stats);
    ESP_LOGD("NVS", "Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)", nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
    return err;
}

esp_err_t read_nvs_menu()
{
    // Open
    nvs_handle_t nvs_storage;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_storage);
    if (err != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        for (int menu_index = 0; menu_index < sizeof(menu) / sizeof(menu_t); menu_index++)
        {
            size_t value_size = sizeof(float);
            esp_err_t read_err = nvs_get_blob(nvs_storage, menu[menu_index].id, &menu[menu_index].val, &value_size);
            switch (read_err)
            {
            case ESP_OK:
                ESP_LOGD("NVS", "Read \"%s\" = %f", menu[menu_index].name, menu[menu_index].val);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGD("NVS", "The value  \"%s\" is not initialized yet!", menu[menu_index].name);
                break;
            default:
                ESP_LOGE("NVS", "Error (%s) reading!", esp_err_to_name(read_err));
            }
        }

        // Close
        nvs_close(nvs_storage);
    }
    return err;
}

int get_menu_index_by_id(const char *id)
{
    int id_length = strlen(id);
    for (int menu_index = 0; menu_index < sizeof(menu) / sizeof(menu_t); menu_index++)
    {
        int entry_id_length = strlen(menu[menu_index].id);
        if (id_length == entry_id_length && strncmp(id, menu[menu_index].id, entry_id_length) == 0)
            return menu_index;
    }
    return -1;
}

float get_menu_val_by_id(const char *id)
{
    for (int menu_index = 0; menu_index < sizeof(menu) / sizeof(menu_t); menu_index++)
    {
        if (strcmp(id, menu[menu_index].id) == 0)
        {
            float value = menu[menu_index].val;
            if (!isfinite(value))
                value = menu[menu_index].min;
            if (value < menu[menu_index].min)
                value = menu[menu_index].min;
            else if (value > menu[menu_index].max)
                value = menu[menu_index].max;
            return value;
        }
    }
    return 0;
}

esp_err_t set_menu_val_by_id(const char *id, float value)
{
    int menu_index = get_menu_index_by_id(id);
    if (menu_index < 0)
        return ESP_ERR_NOT_FOUND;

    if (!is_menu_value_valid(menu_index, value))
    {
        ESP_LOGW("NVS", "Rejected \"%s\" = %f (min %f, max %f)", id, value, menu[menu_index].min, menu[menu_index].max);
        return ESP_ERR_INVALID_ARG;
    }

    if (menu[menu_index].val == value)
        return ESP_OK;

    nvs_handle_t nvs_storage;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_storage);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD("NVS", "Write  \"%s\" : \"%f\"", menu[menu_index].id, value);
    err = nvs_set_blob(nvs_storage, id, &value, sizeof(float));
    if (err == ESP_OK)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        err = nvs_commit(nvs_storage);
        if (err == ESP_OK)
        {
            menu[menu_index].val = value;
            parameters_changed_set(menu_index);
        }
        else
        {
            ESP_LOGE("NVS", "Commit failed: %s", esp_err_to_name(err));
        }
    }
    else
    {
        ESP_LOGE("NVS", "Write failed: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_storage);
    return err;
}

static int html_index = 0;

void reset_menu_html(void)
{
    html_index = 0;
}

int build_menu_html(char *output_buffer)
{
    int written = 0;

    if (html_index == 0)
        written = sprintf(output_buffer, "<table>");

    while (html_index < sizeof(menu) / sizeof(menu_t))
    {
        if (written > CONFIG_LWIP_TCP_MSS - 256)
        {
            return written;
        }

        float value = get_menu_val_by_id(menu[html_index].id);

        if (html_index == 1) // MAC
        {
            uint8_t mac_address[6];
            int mac_high = get_menu_val_by_id(menu[html_index].id);
            int mac_low = get_menu_val_by_id(menu[html_index + 1].id);
            mac_address[0] = (mac_high >> 16) & 0xFF;
            mac_address[1] = (mac_high >> 8) & 0xFF;
            mac_address[2] = (mac_high >> 0) & 0xFF;
            mac_address[3] = (mac_low >> 16) & 0xFF;
            mac_address[4] = (mac_low >> 8) & 0xFF;
            mac_address[5] = (mac_low >> 0) & 0xFF;
            written += sprintf(&output_buffer[written], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"" MACSTR "\"/></td><td></td></tr>\n", menu[html_index].id, menu[html_index].name, menu[html_index].id, menu[html_index].id, MAC2STR(mac_address));
        }
        else if (strlen(menu[html_index].name) > 0)
        {
            written += sprintf(&output_buffer[written], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"%g\"/></td><td>%s</td></tr>\n", menu[html_index].id, menu[html_index].name, menu[html_index].id, menu[html_index].id, value, menu[html_index].izm);
        }
        else // hidden
        {
            written += sprintf(&output_buffer[written], "<input type=\"hidden\" id=\"%s\" name=\"%s\" value=\"%g\">", menu[html_index].id, menu[html_index].id, value);
        }

        html_index++;
    }

    if (written > 0)
    {
        written += sprintf(&output_buffer[written], "</table><br>");
    }
    else
    {
        html_index = 0;
    }

    return written;
}

void console_task(void *arg)
{
    uint8_t line_buffer[256];

    int mac_high;
    int mac_low;
    uint8_t mac_address[6];
    int selected_menu_number = 0;

    char *line = (char *)line_buffer;
    int line_length = 0;

    while (1)
    {
        const int input_char = fgetc(stdin);
        if (input_char > 0) // EOF = -1
        {
            if (input_char == '\n')
            {
                line[line_length] = 0;

                const int next_char = fgetc(stdin); // remove CRLF
                if (next_char != EOF && next_char != '\n' && next_char != '\r')
                    ungetc(next_char, stdin);
            }
            else
            {
                if (line_length < (int)sizeof(line_buffer) - 1)
                    line[line_length++] = input_char;
            }
        }
        else
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        if (input_char == '\n')
        {
            // ESP_LOG_BUFFER_HEXDUMP(TAG, line, line_length + 1, ESP_LOG_INFO);
            // ESP_LOGD(TAG, "Read bytes: '%s'", line);
            float value = atoff((const char *)line);
            switch (selected_menu_number)
            {
            case 0:
                switch ((int)value)
                {
                case 0: // Выводим меню
                    ESP_LOGI("menu", "-------------------------------------------");
                    int menu_index = 0;
                    for (menu_index = 0; menu_index < sizeof(menu) / sizeof(menu_t); menu_index++)
                    {
                        if (menu_index == 1) // MAC
                        {
                            mac_high = get_menu_val_by_id(menu[1].id);
                            mac_low = get_menu_val_by_id(menu[2].id);
                            mac_address[0] = (mac_high >> 16) & 0xFF;
                            mac_address[1] = (mac_high >> 8) & 0xFF;
                            mac_address[2] = (mac_high >> 0) & 0xFF;
                            mac_address[3] = (mac_low >> 16) & 0xFF;
                            mac_address[4] = (mac_low >> 8) & 0xFF;
                            mac_address[5] = (mac_low >> 0) & 0xFF;
                            ESP_LOGI("menu", "%2i. %s: " MACSTR, menu_index + 1, menu[menu_index].name, MAC2STR(mac_address));
                        }
                        else if (strlen(menu[menu_index].name) > 0)
                            ESP_LOGI("menu", "%2i. %s: %g %s", menu_index + 1, menu[menu_index].name, get_menu_val_by_id(menu[menu_index].id), menu[menu_index].izm);
                    }

                    ESP_LOGI("menu", "51. DEBUG! DAC1,DAC2 = 100%");
                    ESP_LOGI("menu", "52. DEBUG! DAC1,DAC2 = 50%");
                    ESP_LOGI("menu", "53. DEBUG! DAC1,DAC2 = 0%");
                    ESP_LOGI("menu", "54. FreeRTOS INFO");
                    ESP_LOGI("menu", "55. Reboot");
                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 2: // MAC
                    mac_high = get_menu_val_by_id(menu[1].id);
                    mac_low = get_menu_val_by_id(menu[2].id);
                    mac_address[0] = (mac_high >> 16) & 0xFF;
                    mac_address[1] = (mac_high >> 8) & 0xFF;
                    mac_address[2] = (mac_high >> 0) & 0xFF;
                    mac_address[3] = (mac_low >> 16) & 0xFF;
                    mac_address[4] = (mac_low >> 8) & 0xFF;
                    mac_address[5] = (mac_low >> 0) & 0xFF;

                    ESP_LOGI("menu", "-------------------------------------------");
                    ESP_LOGI("menu", "%2i. %s: " MACSTR ". Введите новое значение: ", (int)value, menu[(int)value - 1].name, MAC2STR(mac_address));
                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 51: // all adc = 255
                    run_stage_set(100);
                    break;
                case 52: // all adc  = 127
                    run_stage_set(101);
                    break;
                case 53: // all adc  = 0
                    run_stage_set(102);
                    break;
                case 54: // FreeRTOS INFO
                    ESP_LOGI("info", "Minimum free memory: %lu bytes", esp_get_minimum_free_heap_size());
                    ESP_LOGI("wifi_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleWifi));
                    ESP_LOGI("adc_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleADC));
                    /// ESP_LOGI("modem_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleNB));
                    // ESP_LOGI("console_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleConsole));
                    /*
                                        char statsbuf[600];
                                        vTaskGetRunTimeStats(statsbuf);
                                        printf(statsbuf);
                    */
                    break;
                case 55: // Reboot
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    esp_restart();
                    break;
                default:
                    if ((int)value > 0 && (int)value <= sizeof(menu) / sizeof(menu_t))
                    {
                        ESP_LOGI("menu", "-------------------------------------------");
                        ESP_LOGI("menu", "%2i. %s: %g %s. Введите новое значение: ", (int)value, menu[(int)value - 1].name, get_menu_val_by_id(menu[(int)value - 1].id), menu[(int)value - 1].izm);
                        ESP_LOGI("menu", "-------------------------------------------");
                    }
                    else
                    {
                        ESP_LOGE("menu", "Err: %g", value);
                    }
                    break;
                }
                break;
            case 2: // MAC ESPNOW!
                if (sscanf((const char *)line_buffer, "%hhx%*[: -]%hhx%*[: -]%hhx%*[: -]%hhx%*[: -]%hhx%*[: -]%hhx",
                           &mac_address[0], &mac_address[1], &mac_address[2], &mac_address[3], &mac_address[4], &mac_address[5]) == 6)
                {
                    mac_high = (mac_address[0] << 16) | (mac_address[1] << 8) | (mac_address[2]);
                    mac_low = (mac_address[3] << 16) | (mac_address[4] << 8) | (mac_address[5]);

                    esp_err_t mac_high_err = set_menu_val_by_id("MAC1", (float)mac_high);
                    esp_err_t mac_low_err = (mac_high_err == ESP_OK) ? set_menu_val_by_id("MAC2", (float)mac_low) : mac_high_err;

                    if (mac_high_err == ESP_OK && mac_low_err == ESP_OK)
                    {
                        ESP_LOGI("NVS", "MAC saved: " MACSTR, MAC2STR(mac_address));
                        if (xHandleWifi)
                            xTaskNotify(xHandleWifi, REBOOT_NOW, eSetValueWithOverwrite);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to save MAC: %s", esp_err_to_name(mac_low_err));
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Error MAC format");
                }
                break;

            default:
                if (selected_menu_number > 0 && selected_menu_number <= sizeof(menu) / sizeof(menu_t)) // selected_menu_number - номер пункта меню, value - value
                {
                    int menu_index = selected_menu_number - 1;
                    esp_err_t err = set_menu_val_by_id(menu[menu_index].id, value);
                    if (err == ESP_OK)
                    {
                        ESP_LOGI("menu", "-------------------------------------------");
                        ESP_LOGI("menu", "%2i. %s: %f %s.", selected_menu_number, menu[menu_index].name, get_menu_val_by_id(menu[menu_index].id), menu[menu_index].izm);
                        ESP_LOGI("menu", "-------------------------------------------");
                    }
                    else
                    {
                        ESP_LOGE("menu", "Rejected value %g: %s", value, esp_err_to_name(err));
                    }
                }
                break;
            }

            if ((selected_menu_number == 0 && value > 0) && (value <= sizeof(menu) / sizeof(menu_t)))
                selected_menu_number = value;
            else
                selected_menu_number = 0;

            line_length = 0;
        } // if (input_char == '\n')
        vTaskDelay(1);
    }
}

const blink_step_t test_blink_loop[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 50},   // step1: turn on LED 50 ms
    {LED_BLINK_HOLD, LED_STATE_OFF, 100}, // step2: turn off LED 100 ms
    {LED_BLINK_LOOP, 0, 0},               // step3: loop from step1
};

const blink_step_t test_blink_loop2[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 100}, // step1: turn on LED 50 ms
    {LED_BLINK_HOLD, LED_STATE_OFF, 50}, // step2: turn off LED 100 ms
    {LED_BLINK_LOOP, 0, 0},              // step3: loop from step1
};

const blink_step_t test_blink_one_time[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 50},   // step1: turn on LED 50 ms
    {LED_BLINK_HOLD, LED_STATE_OFF, 100}, // step2: turn off LED 100 ms
    {LED_BLINK_HOLD, LED_STATE_ON, 150},  // step3: turn on LED 150 ms
    {LED_BLINK_HOLD, LED_STATE_OFF, 100}, // step4: turn off LED 100 ms
    {LED_BLINK_STOP, 0, 0},               // step5: stop blink (off)
};

typedef enum
{
    BLINK_TEST_BLINK_ONE_TIME, /**< test_blink_one_time */
    BLINK_TEST_BLINK_LOOP,     /**< test_blink_loop */
    BLINK_TEST_BLINK_LOOP2,
    BLINK_MAX, /**< INVALID type */
} led_indicator_blink_type_t;

blink_step_t const *led_indicator_blink_lists[] = {
    [BLINK_TEST_BLINK_ONE_TIME] = test_blink_one_time,
    [BLINK_TEST_BLINK_LOOP] = test_blink_loop,
    [BLINK_TEST_BLINK_LOOP2] = test_blink_loop2,
    [BLINK_MAX] = NULL,
};

int wifi_button_mode = 0;
#define WIFI_BUTTON_TIMEOUT_TICKS (5 * 1000 / 20)
int wifi_button_timeout = WIFI_BUTTON_TIMEOUT_TICKS;

static void button_event_cb(void *button_handle, void *user_data)
{
    // iot_button_print_event((button_handle_t)button_handle);
    button_event_t event = iot_button_get_event((button_handle_t)button_handle);
    switch (event)
    {
    case BUTTON_SINGLE_CLICK:
        if (wifi_button_mode != 1)
        {
            wifi_button_mode = 1;
            ESP_ERROR_CHECK(led_indicator_start(status_led_handle, BLINK_TEST_BLINK_LOOP));

            run_stage_set(999);

            if (xHandleWifi)
                xTaskNotify(xHandleWifi, NOTYFY_WIFI_SWITCH | NOTYFY_WIFI, eSetValueWithOverwrite);
        }
        else
        {
            wifi_button_mode = -1;
            ESP_ERROR_CHECK(led_indicator_stop(status_led_handle, BLINK_TEST_BLINK_LOOP));

            if (xHandleWifi)
                xTaskNotify(xHandleWifi, REBOOT_NOW, eSetValueWithOverwrite);
        }

        wifi_button_timeout = WIFI_BUTTON_TIMEOUT_TICKS;
        break;
    case BUTTON_LONG_PRESS_START:
        break;
    case BUTTON_LONG_PRESS_HOLD:
        break;
    case BUTTON_LONG_PRESS_UP:
        break;
    default:
        break;
    }
}

void button_task(void *arg)
{
    // create gpio button
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = BTN_PIN,
        .active_level = 0,
    };
    button_handle_t btn;
    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn);
    assert(err == ESP_OK);

    err = iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_event_cb, NULL);
    err |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, NULL, button_event_cb, NULL);
    err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, button_event_cb, NULL);
    err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, NULL, button_event_cb, NULL);
    err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, NULL, button_event_cb, NULL);

    led_indicator_gpio_config_t led_gpio_config = {
        .is_active_level_high = 1,
        .gpio_num = LED_PIN, /**< num of GPIO */
    };

    led_indicator_config_t led_config = {
        .blink_lists = led_indicator_blink_lists,
        .blink_list_num = BLINK_MAX,
    };

    ESP_ERROR_CHECK(led_indicator_new_gpio_device(&led_config, &led_gpio_config, &status_led_handle));
    assert(status_led_handle != NULL);

    while (1)
    {
        // timeout key mode
        if (wifi_button_mode != 0)
        {
            /*
                        if (wifi_button_timeout-- <= 0)
                        {
                            ESP_ERROR_CHECK(led_indicator_stop(status_led_handle, BLINK_TEST_BLINK_LOOP));
                            ESP_ERROR_CHECK(led_indicator_set_on_off(status_led_handle, 0));
                        }
            */
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
