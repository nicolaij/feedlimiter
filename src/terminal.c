#include "main.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "driver/gpio.h"
#include "button_gpio.h"
#include "iot_button.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "led_indicator_gpio.h"

static const char *TAG = "terminal";

nvs_handle_t my_handle;

static led_indicator_handle_t led_handle_0 = NULL;

float setup_current = 25.0;
int setup_current_changed = 0;

menu_t menu[] = {
    {.id = "elcurrent", .name = "Задание", .izm = "А", .val = 25, .min = 5, .max = 99},
    {.id = "Kcalc", .name = "К масшт. ADC -> I", .izm = "*1/10000", .val = 200, .min = 1, .max = INT32_MAX},
    {.id = "Kdispl", .name = "К масшт. I -> DAC", .izm = "*1/10000", .val = 50000, .min = 1, .max = INT32_MAX},
    {.id = "pidP", .name = "PID P", .izm = "*1/10000", .val = 1000, .min = 1, .max = INT32_MAX},
    {.id = "pidI", .name = "PID I", .izm = "*1/10000", .val = 10000, .min = 0, .max = INT32_MAX},
    {.id = "pidD", .name = "PID D", .izm = "*1/10000", .val = 0, .min = 0, .max = INT32_MAX},
    {.id = "pidMax", .name = "PID Out maximum", .izm = "", .val = 255, .min = 0, .max = INT32_MAX},
    {.id = "pidMin", .name = "PID Out minimum", .izm = "", .val = 0, .min = 0, .max = INT32_MAX},
};

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
    esp_err_t erro = nvs_open("storage", NVS_READONLY, &my_handle);
    if (erro != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(erro));
    }
    else
    {
        for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
        {
            esp_err_t err = nvs_get_i32(my_handle, menu[i].id, (int32_t *)&menu[i].val);
            switch (err)
            {
            case ESP_OK:
                ESP_LOGD("NVS", "Read \"%s\" = %i", menu[i].name, menu[i].val);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGD("NVS", "The value  \"%s\" is not initialized yet!", menu[i].name);
                break;
            default:
                ESP_LOGE("NVS", "Error (%s) reading!", esp_err_to_name(err));
            }
        }

        // Close
        nvs_close(my_handle);
    }
    return erro;
}

esp_err_t read_nvs_id(const char *key, uint64_t *out_value)
{
    // Open
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("storage", "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        err = nvs_get_u64(my_handle, key, out_value);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD("NVS", "Read \"%s\" = %016llX", key, *out_value);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD("NVS", "The value  \"%s\" is not initialized yet!", key);
            break;
        default:
            ESP_LOGE("NVS", "Error (%s) reading!", esp_err_to_name(err));
        }

        // Close
        nvs_close(my_handle);
    }
    return err;
}

int get_menu_pos_by_id(const char *id)
{
    int ll = strlen(id);
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (ll == l && strncmp(id, menu[i].id, l) == 0)
            return i;
    }
    return -1;
}

int get_menu_val_by_id(const char *id)
{
    int ll = strlen(id);
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (ll == l && strncmp(id, menu[i].id, l) == 0)
            return menu[i].val;
    }
    return 0;
}

esp_err_t set_menu_val_by_id(const char *id, int value)
{
    esp_err_t err = ESP_FAIL;
    int ll = strlen(id);
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        int l = strlen(menu[i].id);
        if (ll == l && strncmp(id, menu[i].id, l) == 0)
        {
            if (menu[i].val != value)
            {
                err = nvs_open("storage", NVS_READWRITE, &my_handle);

                ESP_LOGD("NVS", "Write  \"%s\" : \"%i\"", menu[i].id, value);
                err = nvs_set_i32(my_handle, id, value);
                menu[i].val = value;
                nvs_close(my_handle);
            }
            break;
        }
    }

    return err;
}

int get_menu_json(char *buf)
{
    int pos = 0;
    buf[pos++] = '{';
    for (int i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
    {
        pos += sprintf(&buf[pos], "\"%s\":[\"%s\",%i,\"%s\"]", menu[i].id, menu[i].name, menu[i].val, menu[i].izm);
        if (i < sizeof(menu) / sizeof(menu_t) - 1)
            buf[pos++] = ',';
        else
            buf[pos++] = '}';

        buf[pos] = '\0';
    }
    return pos;
}

int get_menu_html(char *buf)
{
    int pos = 0;
    static int index = 0;

    if (index == 0)
        pos = sprintf(buf, "<table>");

    while (index < sizeof(menu) / sizeof(menu_t))
    {
        if (pos > CONFIG_LWIP_TCP_MSS - 256)
        {
            return pos;
        }

        if (strlen(menu[index].name) > 0)
        {
            pos += sprintf(&buf[pos], "<tr><td><label for=\"%s\">%s:</label></td><td><input type=\"text\" id=\"%s\" name=\"%s\" value=\"%d\"/>%s</td></tr>\n", menu[index].id, menu[index].name, menu[index].id, menu[index].id, menu[index].val, menu[index].izm);
        }
        else // hidden
        {
            pos += sprintf(&buf[pos], "<input type=\"hidden\" id=\"%s\" name=\"%s\" value=\"%d\">", menu[index].id, menu[index].id, menu[index].val);
        }

        index++;
    }

    if (pos > 0)
    {
        pos += sprintf(&buf[pos], "</table><br>");
    }
    else
    {
        index = 0;
    }

    return pos;
}

void console_task(void *arg)
{
    uint8_t serialbuffer[256];

    int selected_menu_id = 0;

    char *data = (char *)serialbuffer;
    int pos = 0;

    while (1)
    {
        const int c = fgetc(stdin);
        if (c > 0) // EOF = -1
        {
            if (c == '\n')
            {
                data[pos] = 0;

                const int nc = fgetc(stdin); // remove CRLF
                if (nc != '\n' && nc != '\r')
                    ungetc(nc, stdin);
            }
            else
            {
                if (pos < sizeof(serialbuffer))
                    data[pos++] = c;
            }
        }
        else
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        if (c == '\n')
        {
            // ESP_LOG_BUFFER_HEXDUMP(TAG, data, pos + 1, ESP_LOG_INFO);
            // ESP_LOGD(TAG, "Read bytes: '%s'", data);
            int n = atoi((const char *)data);
            switch (selected_menu_id)
            {
            case 0:
                switch (n)
                {
                case 0: // Выводим меню
                    ESP_LOGI("menu", "-------------------------------------------");
                    int i = 0;
                    for (i = 0; i < sizeof(menu) / sizeof(menu_t); i++)
                    {
                        if (strlen(menu[i].name) > 0)
                            ESP_LOGI("menu", "%2i. %s: %i %s", i + 1, menu[i].name, menu[i].val, menu[i].izm);
                    }

                    ESP_LOGI("menu", "54. FreeRTOS INFO");
                    ESP_LOGI("menu", "55. Reboot");
                    ESP_LOGI("menu", "-------------------------------------------");
                    break;
                case 54: // FreeRTOS INFO
                    ESP_LOGI("info", "Minimum free memory: %lu bytes", esp_get_minimum_free_heap_size());
                    // ESP_LOGI("wifi_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleWifi));
                    // ESP_LOGI("adc_task", "Task watermark: %d bytes", uxTaskGetStackHighWaterMark(xHandleADC));
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
                    if (n > 0 && n <= sizeof(menu) / sizeof(menu_t))
                    {
                        ESP_LOGI("menu", "-------------------------------------------");
                        ESP_LOGI("menu", "%2i. %s: %i %s. Введите новое значение: ", n, menu[n - 1].name, menu[n - 1].val, menu[n - 1].izm);
                        ESP_LOGI("menu", "-------------------------------------------");
                    }
                    else
                    {
                        ESP_LOGE("menu", "Err: %i", n);
                    }
                    break;
                }
                break;

            default:
                if (selected_menu_id > 0 && selected_menu_id <= sizeof(menu) / sizeof(menu_t)) // selected_menu_id - номер пункта меню, n - value
                {
                    if (n >= menu[selected_menu_id - 1].min && n <= menu[selected_menu_id - 1].max)
                    {
                        menu[selected_menu_id - 1].val = n;
                        esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
                        }
                        else
                        {
                            err = nvs_set_i32(my_handle, menu[selected_menu_id - 1].id, menu[selected_menu_id - 1].val);
                            if (err != ESP_OK)
                            {
                                ESP_LOGE(TAG, "%s", esp_err_to_name(err));
                            }
                            else
                            {
                                ESP_LOGI("menu", "-------------------------------------------");
                                ESP_LOGI("menu", "%2i. %s: %i %s.", selected_menu_id, menu[selected_menu_id - 1].name, menu[selected_menu_id - 1].val, menu[selected_menu_id - 1].izm);
                                ESP_LOGI("menu", "-------------------------------------------");
                            }
                        }

                        ESP_LOGD(TAG, "Committing updates in NVS ... ");
                        err = nvs_commit(my_handle);
                        if (err != ESP_OK)
                            ESP_LOGE(TAG, "Committing updates in NVS ... - Failed!");

                        // Close
                        nvs_close(my_handle);
                    }
                }
                break;
            }

            if ((selected_menu_id == 0 && n > 0) && (n < sizeof(menu) / sizeof(menu_t)))
                selected_menu_id = n;
            else
                selected_menu_id = 0;

            pos = 0;
        } // if (c == '\n')
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

int key_dir = 0;
#define KEY_TIMEOUT (5 * 1000 / 20)
int key_timeout = KEY_TIMEOUT;
int setup_set[3] = {25, 10, 50};
int setup_curr = 25;
int setup_cntr = 0;

static void button_event_cb(void *arg, void *data)
{
    // iot_button_print_event((button_handle_t)arg);
    button_event_t event = iot_button_get_event((button_handle_t)arg);
    switch (event)
    {
    case BUTTON_SINGLE_CLICK:
        if (key_dir != 1)
        {
            key_dir = 1;
            ESP_ERROR_CHECK(led_indicator_stop(led_handle_0, BLINK_TEST_BLINK_LOOP2));
            ESP_ERROR_CHECK(led_indicator_start(led_handle_0, BLINK_TEST_BLINK_LOOP));
        }
        else
        {
            key_dir = -1;
            ESP_ERROR_CHECK(led_indicator_stop(led_handle_0, BLINK_TEST_BLINK_LOOP));
            ESP_ERROR_CHECK(led_indicator_start(led_handle_0, BLINK_TEST_BLINK_LOOP2));
        }

        ESP_LOGI(TAG, "Set DIR; %d", key_dir);

        key_timeout = KEY_TIMEOUT;
        break;
    case BUTTON_LONG_PRESS_START:
        ESP_ERROR_CHECK(led_indicator_stop(led_handle_0, BLINK_TEST_BLINK_LOOP));
        ESP_ERROR_CHECK(led_indicator_stop(led_handle_0, BLINK_TEST_BLINK_LOOP2));
        ESP_ERROR_CHECK(led_indicator_set_on_off(led_handle_0, 1));
        break;
    case BUTTON_LONG_PRESS_HOLD:
        setup_cntr++;
        key_timeout = KEY_TIMEOUT;
        if (setup_cntr % 20 == 0)
        {
            setup_curr += key_dir;

            ESP_LOGI(TAG, "Set; %d", setup_curr);
        }
        break;
    case BUTTON_LONG_PRESS_UP:
        key_timeout = KEY_TIMEOUT;
        ESP_ERROR_CHECK(led_indicator_set_on_off(led_handle_0, 0));

        if (key_dir == 1)
        {
            ESP_ERROR_CHECK(led_indicator_start(led_handle_0, BLINK_TEST_BLINK_LOOP));
        }
        if (key_dir == -1)
        {
            ESP_ERROR_CHECK(led_indicator_start(led_handle_0, BLINK_TEST_BLINK_LOOP2));
        }
        break;
    default:
        break;
    }
}

void btn_task(void *arg)
{

    // create gpio button
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = BTN_PIN,
        .active_level = 0,
    };
    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn);
    assert(ret == ESP_OK);

    ret = iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_event_cb, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, NULL, button_event_cb, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, button_event_cb, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, NULL, button_event_cb, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, NULL, button_event_cb, NULL);
    // 1 - режим настройки, 2 - изменение настройки
    int state = 0;
    int led_blink = 0;

    led_indicator_gpio_config_t led_indicator_gpio_config = {
        .is_active_level_high = 1,
        .gpio_num = LED_PIN, /**< num of GPIO */
    };

    led_indicator_config_t config = {
        .blink_lists = led_indicator_blink_lists,
        .blink_list_num = BLINK_MAX,
    };

    ESP_ERROR_CHECK(led_indicator_new_gpio_device(&config, &led_indicator_gpio_config, &led_handle_0));
    assert(led_handle_0 != NULL);

    setup_curr = menu[0].val;
    setup_current = setup_curr;

    while (1)
    {
        // timeout key mode
        if (key_dir != 0)
        {
            setup_current_changed = setup_curr;
            if (key_timeout-- <= 0)
            {
                ESP_ERROR_CHECK(led_indicator_stop(led_handle_0, BLINK_TEST_BLINK_LOOP));
                ESP_ERROR_CHECK(led_indicator_stop(led_handle_0, BLINK_TEST_BLINK_LOOP2));
                ESP_ERROR_CHECK(led_indicator_set_on_off(led_handle_0, 0));

                key_dir = 0;
                set_menu_val_by_id("elcurrent", setup_curr);
                setup_current = setup_curr;
                setup_current_changed = 0;
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
