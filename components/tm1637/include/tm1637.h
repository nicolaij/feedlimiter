/**
 * @file tm1637.h
 * @brief TM1637 7-segment display driver for ESP-IDF
 *
 * Copyright (c) 2025 Siratul Islam
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef TM1637_H
#define TM1637_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Maximum number of digits supported
 */
#define TM1637_MAX_DIGITS 4

/**
 * @brief Maximum brightness level (0-7)
 */
#define TM1637_BRIGHTNESS_MAX 7

/**
 * @brief Segment definitions for manual control
 */
#define TM1637_SEG_A 0x01
#define TM1637_SEG_B 0x02
#define TM1637_SEG_C 0x04
#define TM1637_SEG_D 0x08
#define TM1637_SEG_E 0x10
#define TM1637_SEG_F 0x20
#define TM1637_SEG_G 0x40
#define TM1637_SEG_DP 0x80

    /**
     * @brief TM1637 device handle
     */
    typedef struct tm1637_dev_t *tm1637_handle_t;

    /**
     * @brief TM1637 configuration structure
     */
    typedef struct
    {
        gpio_num_t clk_pin;    /**< Clock pin GPIO number */
        gpio_num_t dio_pin;    /**< Data I/O pin GPIO number */
        uint32_t bit_delay_us; /**< Bit delay in microseconds (default: 100) */
    } tm1637_config_t;

    /**
     * @brief Initialize TM1637 device
     *
     * @param config Configuration structure
     * @param handle Output handle for the initialized device
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_init(const tm1637_config_t *config, tm1637_handle_t *handle);

    /**
     * @brief Deinitialize TM1637 device and free resources
     *
     * @param handle Device handle
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_deinit(tm1637_handle_t handle);

    /**
     * @brief Set brightness and display on/off
     *
     * @param handle Device handle
     * @param brightness Brightness level (0-7)
     * @param on Turn display on (true) or off (false)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_set_brightness(tm1637_handle_t handle, uint8_t brightness, bool on);

    /**
     * @brief Turn display on
     *
     * @param handle Device handle
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_display_on(tm1637_handle_t handle);

    /**
     * @brief Turn display off
     *
     * @param handle Device handle
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_display_off(tm1637_handle_t handle);

    /**
     * @brief Set raw segment data
     *
     * @param handle Device handle
     * @param segments Array of segment data
     * @param length Number of segments to write
     * @param position Starting position (0-3)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_set_segments(tm1637_handle_t handle, const uint8_t *segments,
                                  uint8_t length, uint8_t position);

    /**
     * @brief Clear the display
     *
     * @param handle Device handle
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_clear(tm1637_handle_t handle);

    /**
     * @brief Display a decimal number
     *
     * @param handle Device handle
     * @param number Number to display
     * @param leading_zeros Show leading zeros
     * @param length Number of digits to use
     * @param position Starting position (0-3)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_show_number(tm1637_handle_t handle, int32_t number,
                                 bool leading_zeros, uint8_t length, uint8_t position);

    /**
     * @brief Display a hexadecimal number
     *
     * @param handle Device handle
     * @param number Number to display
     * @param leading_zeros Show leading zeros
     * @param length Number of digits to use
     * @param position Starting position (0-3)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_show_number_hex(tm1637_handle_t handle, uint32_t number,
                                     bool leading_zeros, uint8_t length, uint8_t position);

    /**
     * @brief Write string to display (alphanumeric support)
     *
     * @param handle Device handle
     * @param text Text string to display (supports 0-9, A-Z, a-z, space, minus, decimal point)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_write_string(tm1637_handle_t handle, const char *text);

    /**
     * @brief Scroll text across the display
     *
     * This function scrolls the given text from right to left across the display.
     * It blocks until the entire text has scrolled through.
     *
     * @param handle Device handle
     * @param text Text string to scroll
     * @param delay_ms Delay in milliseconds between each scroll step (recommended: 200-500ms)
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_scroll_text(tm1637_handle_t handle, const char *text, uint32_t delay_ms);

    /**
     * @brief Start scrolling text in the background (non-blocking)
     *
     * Creates a task to continuously scroll text. The text will loop indefinitely
     * until tm1637_scroll_stop() is called.
     *
     * @param handle Device handle
     * @param text Text string to scroll (will be copied internally)
     * @param delay_ms Delay in milliseconds between each scroll step
     * @param loop If true, text scrolls continuously; if false, scrolls once and stops
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_scroll_start(tm1637_handle_t handle, const char *text,
                                  uint32_t delay_ms, bool loop);

    /**
     * @brief Stop background scrolling text
     *
     * @param handle Device handle
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t tm1637_scroll_stop(tm1637_handle_t handle);

    /**
     * @brief Encode a single digit (0-15) to 7-segment pattern
     *
     * @param digit Digit to encode (0-15)
     * @return 7-segment pattern byte
     */
    uint8_t tm1637_encode_digit(uint8_t digit);

#ifdef __cplusplus
}
#endif

#endif /* TM1637_H */