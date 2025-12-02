/**
 * @file main.c
 * @brief Basic TM1637 display example
 *
 * This example demonstrates basic usage of the TM1637 driver:
 * - Initializing the display
 * - Setting brightness
 * - Displaying numbers
 * - Displaying text
 * - Clearing the display
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tm1637.h"

static const char *TAG = "EXAMPLE";

// Define your GPIO pins
#define TM1637_CLK_PIN GPIO_NUM_18
#define TM1637_DIO_PIN GPIO_NUM_19

void app_main(void)
{
    ESP_LOGI(TAG, "TM1637 Basic Display Example");

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

    // Example 2: Display number with leading zeros
    ESP_LOGI(TAG, "Displaying number with leading zeros: 0042");
    tm1637_show_number(display, 42, true, 4, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Example 3: Display negative number
    ESP_LOGI(TAG, "Displaying negative number: -123");
    tm1637_show_number(display, -123, false, 4, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Example 4: Display hexadecimal
    ESP_LOGI(TAG, "Displaying hex: CAFE");
    tm1637_show_number_hex(display, 0xCAFE, false, 4, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Example 5: Display text
    ESP_LOGI(TAG, "Displaying text: HELP");
    tm1637_write_string(display, "HELP");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Example 6: Display text with decimal point
    ESP_LOGI(TAG, "Displaying: 12.5");
    tm1637_write_string(display, "12.5");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Example 7: Brightness test
    ESP_LOGI(TAG, "Testing brightness levels");
    for (uint8_t level = 0; level <= 7; level++)
    {
        tm1637_set_brightness(display, level, true);
        tm1637_show_number(display, level, false, 4, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Example 8: Display on/off
    ESP_LOGI(TAG, "Testing display on/off");
    for (int i = 0; i < 5; i++)
    {
        tm1637_display_off(display);
        vTaskDelay(pdMS_TO_TICKS(300));
        tm1637_display_on(display);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    // Example 9: Counter
    ESP_LOGI(TAG, "Starting counter...");
    for (int count = 0; count < 100; count++)
    {
        tm1637_show_number(display, count, false, 4, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Clear display
    ESP_LOGI(TAG, "Clearing display");
    tm1637_clear(display);

    // Clean up
    tm1637_deinit(display);
    ESP_LOGI(TAG, "Example completed");
}