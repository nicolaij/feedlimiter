/**
 * @file main.c
 * @brief TM1637 text scrolling example
 *
 * This example demonstrates text scrolling features:
 * - Blocking scroll (tm1637_scroll_text)
 * - Non-blocking continuous scroll (tm1637_scroll_start/stop)
 * - Combining scrolling with other tasks
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tm1637.h"

static const char *TAG = "SCROLL_EXAMPLE";

// Define your GPIO pins
#define TM1637_CLK_PIN GPIO_NUM_18
#define TM1637_DIO_PIN GPIO_NUM_19

void app_main(void)
{
    ESP_LOGI(TAG, "TM1637 Scrolling Text Example");

    // Configure and initialize display
    tm1637_config_t config = {
        .clk_pin = TM1637_CLK_PIN,
        .dio_pin = TM1637_DIO_PIN,
        .bit_delay_us = 100};

    tm1637_handle_t display;
    esp_err_t ret = tm1637_init(&config, &display);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize TM1637: %s", esp_err_to_name(ret));
        return;
    }

    // Set brightness
    tm1637_set_brightness(display, 5, true);

    // Example 1: Blocking scroll - waits until complete
    ESP_LOGI(TAG, "Example 1: Blocking scroll");
    tm1637_scroll_text(display, "HELLO WORLD", 300);
    ESP_LOGI(TAG, "Scroll completed");

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Example 2: Blocking scroll with faster speed
    ESP_LOGI(TAG, "Example 2: Fast scroll");
    tm1637_scroll_text(display, "ESP32 TM1637 DRIVER", 200);

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Example 3: Non-blocking continuous scroll
    ESP_LOGI(TAG, "Example 3: Continuous background scroll");
    tm1637_scroll_start(display, "TEMPERATURE 25.6C HUMIDITY 60PCT", 250, true);

    // Do other work while scrolling in background
    ESP_LOGI(TAG, "Display is scrolling in background...");
    for (int i = 0; i < 10; i++)
    {
        ESP_LOGI(TAG, "Other task work: %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Stop the background scroll
    ESP_LOGI(TAG, "Stopping scroll");
    tm1637_scroll_stop(display);

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Example 4: One-time non-blocking scroll
    ESP_LOGI(TAG, "Example 4: One-time scroll (no loop)");
    tm1637_scroll_start(display, "WELCOME TO ESP32", 300, false);

    // Wait for it to finish
    vTaskDelay(pdMS_TO_TICKS(8000));
    tm1637_scroll_stop(display); // Safe to call even if already stopped

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Example 5: Multiple scrolling messages
    ESP_LOGI(TAG, "Example 5: Multiple messages");

    const char *messages[] = {
        "MSG 1 FIRST",
        "MSG 2 SECOND",
        "MSG 3 THIRD",
        "DONE"};

    for (int i = 0; i < 4; i++)
    {
        ESP_LOGI(TAG, "Scrolling message %d", i + 1);
        tm1637_scroll_text(display, messages[i], 250);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Example 6: Scrolling with decimal points
    ESP_LOGI(TAG, "Example 6: Scroll with decimals");
    tm1637_scroll_text(display, "V1.2.5 BUILD 2025", 300);

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Final display
    ESP_LOGI(TAG, "Example complete - showing END");
    tm1637_write_string(display, "END");

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Clean up
    tm1637_clear(display);
    tm1637_deinit(display);
    ESP_LOGI(TAG, "Example completed");
}