/**
 * @file tm1637.c
 * @brief TM1637 7-segment display driver implementation for ESP-IDF
 *
 * Copyright (c) 2025 Siratul Islam
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "tm1637.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

static const char *TAG = "TM1637";

/* TM1637 Command Set (from datasheet) */
#define CMD_SET_DATA 0x40     /* Data command setting */
#define CMD_SET_ADDR 0xC0     /* Address command setting */
#define CMD_DISPLAY_CTRL 0x88 /* Display control command (ON + brightness) */

/* Data command bits */
#define DATA_WRITE_MODE 0x00 /* Write data to display register */
#define DATA_AUTO_INC 0x00   /* Automatic address increment */
#define DATA_FIXED_ADDR 0x04 /* Fixed address (no auto-increment) */

/* Display control bits */
#define DISPLAY_ON_BIT 0x08  /* Display ON */
#define BRIGHTNESS_MASK 0x07 /* Brightness level mask (0-7) */

/* Default timing (from datasheet requirements) */
#define DEFAULT_BIT_DELAY_US 100
#define TIMING_START_US 2
#define TIMING_STOP_US 2

/**
 * @brief Segment lookup table for hexadecimal digits (0-F)
 * Based on standard 7-segment encoding
 * Bit layout: DP G F E D C B A
 */
static const uint8_t SEGMENT_LOOKUP[16] = {
    0x3F, /* 0: segments A B C D E F */
    0x06, /* 1: segments B C */
    0x5B, /* 2: segments A B D E G */
    0x4F, /* 3: segments A B C D G */
    0x66, /* 4: segments B C F G */
    0x6D, /* 5: segments A C D F G */
    0x7D, /* 6: segments A C D E F G */
    0x07, /* 7: segments A B C */
    0x7F, /* 8: segments A B C D E F G */
    0x6F, /* 9: segments A B C D F G */
    0x77, /* A: segments A B C E F G */
    0x7C, /* b: segments C D E F G */
    0x39, /* C: segments A D E F */
    0x5E, /* d: segments B C D E G */
    0x79, /* E: segments A D E F G */
    0x71, /* F: segments A E F G */
};

/**
 * @brief Extended character lookup for alphanumeric display
 */
static const uint8_t CHAR_LOOKUP[26] = {
    0x77, /* A */ 0x7C, /* b */ 0x39, /* C */ 0x5E, /* d */
    0x79, /* E */ 0x71, /* F */ 0x3D, /* G */ 0x76, /* H */
    0x30, /* I */ 0x1E, /* J */ 0x00, /* K */ 0x38, /* L */
    0x00, /* M */ 0x37, /* n */ 0x3F, /* O */ 0x73, /* P */
    0x67, /* q */ 0x50, /* r */ 0x6D, /* S */ 0x78, /* t */
    0x3E, /* U */ 0x00, /* V */ 0x00, /* W */ 0x76, /* X */
    0x6E, /* y */ 0x5B,                             /* Z */
};

#define MINUS_SEGMENT 0x40 /* G segment for minus sign */
#define SPACE_SEGMENT 0x00 /* All segments off */

/**
 * @brief Device structure
 */
struct tm1637_dev_t
{
    gpio_num_t pin_clk;
    gpio_num_t pin_dio;
    uint32_t delay_us;
    uint8_t brightness_level;
    bool is_on;
    uint8_t buffer[TM1637_MAX_DIGITS];
    SemaphoreHandle_t mutex;

    /* Scrolling text support */
    TaskHandle_t scroll_task;
    char *scroll_text;
    uint32_t scroll_delay_ms;
    bool scroll_loop;
    bool scroll_running;
};

/* === Protocol Layer: Hardware Communication === */

static inline void delay_us(tm1637_handle_t dev)
{
    ets_delay_us(dev->delay_us);
}

/**
 * @brief Generate START condition
 * DIO falls while CLK is high
 */
static void protocol_start(tm1637_handle_t dev)
{
    /* Ensure both pins are high initially */
    gpio_set_direction(dev->pin_dio, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->pin_clk, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->pin_dio, 1);
    gpio_set_level(dev->pin_clk, 1);
    delay_us(dev);

    /* START: DIO falls while CLK high */
    gpio_set_level(dev->pin_dio, 0);
    ets_delay_us(TIMING_START_US);
    gpio_set_level(dev->pin_clk, 0);
    delay_us(dev);
}

/**
 * @brief Generate STOP condition
 * DIO rises while CLK is high
 */
static void protocol_stop(tm1637_handle_t dev)
{
    /* CLK low, DIO low */
    gpio_set_direction(dev->pin_clk, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->pin_dio, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->pin_clk, 0);
    gpio_set_level(dev->pin_dio, 0);
    delay_us(dev);

    /* CLK high */
    gpio_set_level(dev->pin_clk, 1);
    delay_us(dev);

    /* STOP: DIO rises while CLK high */
    gpio_set_level(dev->pin_dio, 1);
    ets_delay_us(TIMING_STOP_US);
}

/**
 * @brief Transmit one byte with ACK checking
 * LSB first transmission as per TM1637 protocol
 */
static bool protocol_transmit_byte(tm1637_handle_t dev, uint8_t byte_data)
{
    gpio_set_direction(dev->pin_clk, GPIO_MODE_OUTPUT);

    /* Send 8 bits, LSB first */
    for (uint8_t bit_idx = 0; bit_idx < 8; bit_idx++)
    {
        gpio_set_level(dev->pin_clk, 0);
        delay_us(dev);

        /* Set DIO based on current bit */
        uint8_t bit_val = (byte_data >> bit_idx) & 0x01;
        if (bit_val)
        {
            /* High: release DIO (pulled high) */
            gpio_set_direction(dev->pin_dio, GPIO_MODE_INPUT);
        }
        else
        {
            /* Low: drive DIO low */
            gpio_set_direction(dev->pin_dio, GPIO_MODE_OUTPUT);
            gpio_set_level(dev->pin_dio, 0);
        }
        delay_us(dev);

        gpio_set_level(dev->pin_clk, 1);
        delay_us(dev);
    }

    /* Read ACK bit (9th clock) */
    gpio_set_level(dev->pin_clk, 0);
    gpio_set_direction(dev->pin_dio, GPIO_MODE_INPUT);
    delay_us(dev);

    gpio_set_level(dev->pin_clk, 1);
    delay_us(dev);

    bool ack_received = (gpio_get_level(dev->pin_dio) == 0);

    gpio_set_level(dev->pin_clk, 0);
    delay_us(dev);

    return ack_received;
}

/**
 * @brief Refresh display with current buffer contents
 */
static esp_err_t hardware_update(tm1637_handle_t dev)
{
    if (!dev)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Step 1: Send data write command */
    protocol_start(dev);
    protocol_transmit_byte(dev, CMD_SET_DATA | DATA_AUTO_INC);
    protocol_stop(dev);

    /* Step 2: Send address and data */
    protocol_start(dev);
    protocol_transmit_byte(dev, CMD_SET_ADDR | 0x00); /* Start at address 0 */

    for (uint8_t i = 0; i < TM1637_MAX_DIGITS; i++)
    {
        protocol_transmit_byte(dev, dev->buffer[i]);
    }
    protocol_stop(dev);

    /* Step 3: Send display control command */
    uint8_t ctrl_byte = CMD_DISPLAY_CTRL;
    ctrl_byte |= (dev->brightness_level & BRIGHTNESS_MASK);
    if (dev->is_on)
    {
        ctrl_byte |= DISPLAY_ON_BIT;
    }

    protocol_start(dev);
    protocol_transmit_byte(dev, ctrl_byte);
    protocol_stop(dev);

    return ESP_OK;
}

/* === Public API Implementation === */

esp_err_t tm1637_init(const tm1637_config_t *config, tm1637_handle_t *handle)
{
    if (!config || !handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    tm1637_handle_t dev = calloc(1, sizeof(struct tm1637_dev_t));
    if (!dev)
    {
        ESP_LOGE(TAG, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }

    dev->pin_clk = config->clk_pin;
    dev->pin_dio = config->dio_pin;
    dev->delay_us = (config->bit_delay_us > 0) ? config->bit_delay_us : DEFAULT_BIT_DELAY_US;
    dev->brightness_level = TM1637_BRIGHTNESS_MAX;
    dev->is_on = true;
    dev->scroll_task = NULL;
    dev->scroll_text = NULL;
    dev->scroll_running = false;

    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "Mutex creation failed");
        free(dev);
        return ESP_ERR_NO_MEM;
    }

    /* Configure GPIO pins */
    gpio_config_t io_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    io_cfg.pin_bit_mask = (1ULL << dev->pin_clk);
    esp_err_t ret = gpio_config(&io_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "CLK GPIO configuration failed");
        vSemaphoreDelete(dev->mutex);
        free(dev);
        return ret;
    }

    io_cfg.pin_bit_mask = (1ULL << dev->pin_dio);
    ret = gpio_config(&io_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "DIO GPIO configuration failed");
        vSemaphoreDelete(dev->mutex);
        free(dev);
        return ret;
    }

    /* Initialize pins to idle state */
    gpio_set_level(dev->pin_clk, 0);
    gpio_set_level(dev->pin_dio, 0);

    /* Clear display */
    memset(dev->buffer, 0, sizeof(dev->buffer));
    ret = hardware_update(dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Initial display update failed");
        vSemaphoreDelete(dev->mutex);
        free(dev);
        return ret;
    }

    *handle = dev;
    ESP_LOGI(TAG, "Device initialized on CLK=%d, DIO=%d", dev->pin_clk, dev->pin_dio);
    return ESP_OK;
}

esp_err_t tm1637_deinit(tm1637_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Stop scrolling if active */
    tm1637_scroll_stop(handle);

    tm1637_clear(handle);

    if (handle->mutex)
    {
        vSemaphoreDelete(handle->mutex);
    }

    free(handle);
    return ESP_OK;
}

esp_err_t tm1637_set_brightness(tm1637_handle_t handle, uint8_t brightness, bool on)
{
    if (!handle || brightness > TM1637_BRIGHTNESS_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    handle->brightness_level = brightness;
    handle->is_on = on;
    esp_err_t ret = hardware_update(handle);

    xSemaphoreGive(handle->mutex);
    return ret;
}

esp_err_t tm1637_display_on(tm1637_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return tm1637_set_brightness(handle, handle->brightness_level, true);
}

esp_err_t tm1637_display_off(tm1637_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return tm1637_set_brightness(handle, handle->brightness_level, false);
}

esp_err_t tm1637_set_segments(tm1637_handle_t handle, const uint8_t *segments,
                              uint8_t length, uint8_t position)
{
    if (!handle || !segments || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (position >= TM1637_MAX_DIGITS || (position + length) > TM1637_MAX_DIGITS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(&handle->buffer[position], segments, length);
    esp_err_t ret = hardware_update(handle);

    xSemaphoreGive(handle->mutex);
    return ret;
}

esp_err_t tm1637_clear(tm1637_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    memset(handle->buffer, 0, sizeof(handle->buffer));
    esp_err_t ret = hardware_update(handle);

    xSemaphoreGive(handle->mutex);
    return ret;
}

uint8_t tm1637_encode_digit(uint8_t digit)
{
    return SEGMENT_LOOKUP[digit & 0x0F];
}

esp_err_t tm1637_show_number(tm1637_handle_t handle, int32_t number,
                             bool leading_zeros, uint8_t length, uint8_t position)
{
    if (!handle || length == 0 || length > TM1637_MAX_DIGITS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (position >= TM1637_MAX_DIGITS || (position + length) > TM1637_MAX_DIGITS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t digit_buffer[TM1637_MAX_DIGITS] = {0};
    bool is_negative = (number < 0);
    uint32_t abs_number = is_negative ? -number : number;

    /* Convert number to individual digits */
    uint8_t digit_count = 0;
    uint32_t temp = abs_number;

    do
    {
        digit_count++;
        temp /= 10;
    } while (temp > 0 && digit_count < length);

    /* Build display buffer from right to left */
    temp = abs_number;
    for (int8_t i = length - 1; i >= 0; i--)
    {
        uint8_t digit = temp % 10;

        if (temp == 0 && i < (length - 1) && !leading_zeros)
        {
            /* Leading zero - leave blank unless it's the last digit */
            digit_buffer[i] = 0;
        }
        else
        {
            digit_buffer[i] = SEGMENT_LOOKUP[digit];
        }

        temp /= 10;
    }

    /* Add minus sign for negative numbers */
    if (is_negative)
    {
        /* Find first non-zero position and place minus sign before it */
        for (int8_t i = 0; i < length; i++)
        {
            if (digit_buffer[i] == 0 || i == 0)
            {
                digit_buffer[i] = MINUS_SEGMENT;
                break;
            }
        }
    }

    return tm1637_set_segments(handle, digit_buffer, length, position);
}

esp_err_t tm1637_show_number_hex(tm1637_handle_t handle, uint32_t number,
                                 bool leading_zeros, uint8_t length, uint8_t position)
{
    if (!handle || length == 0 || length > TM1637_MAX_DIGITS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (position >= TM1637_MAX_DIGITS || (position + length) > TM1637_MAX_DIGITS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t digit_buffer[TM1637_MAX_DIGITS] = {0};

    /* Extract hex digits from right to left */
    for (int8_t i = length - 1; i >= 0; i--)
    {
        uint8_t nibble = number & 0x0F;

        if (number == 0 && i < (length - 1) && !leading_zeros)
        {
            digit_buffer[i] = 0;
        }
        else
        {
            digit_buffer[i] = SEGMENT_LOOKUP[nibble];
        }

        number >>= 4;
    }

    return tm1637_set_segments(handle, digit_buffer, length, position);
}

esp_err_t tm1637_write_string(tm1637_handle_t handle, const char *text)
{
    if (!handle || !text)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    memset(handle->buffer, 0, sizeof(handle->buffer));

    uint8_t display_pos = 0;
    size_t text_len = strlen(text);

    for (size_t i = 0; i < text_len && display_pos < TM1637_MAX_DIGITS; i++)
    {
        char ch = text[i];
        uint8_t segment_data = 0;
        bool char_valid = false;

        /* Check for decimal point modifier */
        bool has_dp = (i + 1 < text_len && text[i + 1] == '.');

        if (ch >= '0' && ch <= '9')
        {
            segment_data = SEGMENT_LOOKUP[ch - '0'];
            char_valid = true;
        }
        else if (ch >= 'A' && ch <= 'Z')
        {
            segment_data = CHAR_LOOKUP[ch - 'A'];
            char_valid = true;
        }
        else if (ch >= 'a' && ch <= 'z')
        {
            segment_data = CHAR_LOOKUP[ch - 'a'];
            char_valid = true;
        }
        else if (ch == '-')
        {
            segment_data = MINUS_SEGMENT;
            char_valid = true;
        }
        else if (ch == ' ')
        {
            segment_data = SPACE_SEGMENT;
            char_valid = true;
        }
        else if (ch == '.')
        {
            /* Skip standalone decimal points */
            continue;
        }

        if (char_valid)
        {
            if (has_dp)
            {
                segment_data |= TM1637_SEG_DP;
                i++; /* Skip the decimal point in next iteration */
            }
            handle->buffer[display_pos++] = segment_data;
        }
    }

    esp_err_t ret = hardware_update(handle);
    xSemaphoreGive(handle->mutex);
    return ret;
}

/* === Scrolling Text Functions === */

/**
 * @brief Convert a character to segment data (internal helper)
 */
static uint8_t char_to_segment(char ch)
{
    if (ch >= '0' && ch <= '9')
    {
        return SEGMENT_LOOKUP[ch - '0'];
    }
    else if (ch >= 'A' && ch <= 'Z')
    {
        return CHAR_LOOKUP[ch - 'A'];
    }
    else if (ch >= 'a' && ch <= 'z')
    {
        return CHAR_LOOKUP[ch - 'a'];
    }
    else if (ch == '-')
    {
        return MINUS_SEGMENT;
    }
    else if (ch == ' ')
    {
        return SPACE_SEGMENT;
    }
    return SPACE_SEGMENT; /* Unknown character displays as space */
}

esp_err_t tm1637_scroll_text(tm1637_handle_t handle, const char *text, uint32_t delay_ms)
{
    if (!handle || !text)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t text_len = strlen(text);
    if (text_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Add leading and trailing spaces for smooth scroll effect */
    size_t total_len = text_len + TM1637_MAX_DIGITS + TM1637_MAX_DIGITS;
    char *padded_text = malloc(total_len + 1);
    if (!padded_text)
    {
        return ESP_ERR_NO_MEM;
    }

    /* Build padded string: "    TEXT    " */
    memset(padded_text, ' ', total_len);
    memcpy(padded_text + TM1637_MAX_DIGITS, text, text_len);
    padded_text[total_len] = '\0';

    /* Scroll through the text */
    for (size_t pos = 0; pos <= text_len + TM1637_MAX_DIGITS; pos++)
    {
        uint8_t display_buffer[TM1637_MAX_DIGITS];

        /* Extract 4-character window */
        for (uint8_t i = 0; i < TM1637_MAX_DIGITS; i++)
        {
            char ch = padded_text[pos + i];
            display_buffer[i] = char_to_segment(ch);

            /* Check for decimal point */
            if (pos + i + 1 < total_len && padded_text[pos + i + 1] == '.')
            {
                display_buffer[i] |= TM1637_SEG_DP;
            }
        }

        /* Update display */
        if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            memcpy(handle->buffer, display_buffer, TM1637_MAX_DIGITS);
            hardware_update(handle);
            xSemaphoreGive(handle->mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    free(padded_text);
    return ESP_OK;
}

/**
 * @brief Background scrolling task
 */
static void scroll_task_function(void *pvParameters)
{
    tm1637_handle_t handle = (tm1637_handle_t)pvParameters;

    if (!handle || !handle->scroll_text)
    {
        vTaskDelete(NULL);
        return;
    }

    size_t text_len = strlen(handle->scroll_text);
    size_t total_len = text_len + TM1637_MAX_DIGITS + TM1637_MAX_DIGITS;

    /* Build padded string */
    char *padded_text = malloc(total_len + 1);
    if (!padded_text)
    {
        handle->scroll_running = false;
        vTaskDelete(NULL);
        return;
    }

    memset(padded_text, ' ', total_len);
    memcpy(padded_text + TM1637_MAX_DIGITS, handle->scroll_text, text_len);
    padded_text[total_len] = '\0';

    /* Main scroll loop */
    while (handle->scroll_running)
    {
        for (size_t pos = 0; pos <= text_len + TM1637_MAX_DIGITS && handle->scroll_running; pos++)
        {
            uint8_t display_buffer[TM1637_MAX_DIGITS];

            /* Extract 4-character window */
            for (uint8_t i = 0; i < TM1637_MAX_DIGITS; i++)
            {
                char ch = padded_text[pos + i];
                display_buffer[i] = char_to_segment(ch);

                /* Check for decimal point */
                if (pos + i + 1 < total_len && padded_text[pos + i + 1] == '.')
                {
                    display_buffer[i] |= TM1637_SEG_DP;
                }
            }

            /* Update display */
            if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                memcpy(handle->buffer, display_buffer, TM1637_MAX_DIGITS);
                hardware_update(handle);
                xSemaphoreGive(handle->mutex);
            }

            vTaskDelay(pdMS_TO_TICKS(handle->scroll_delay_ms));
        }

        /* If not looping, exit after one pass */
        if (!handle->scroll_loop)
        {
            handle->scroll_running = false;
        }
    }

    free(padded_text);
    handle->scroll_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t tm1637_scroll_start(tm1637_handle_t handle, const char *text,
                              uint32_t delay_ms, bool loop)
{
    if (!handle || !text)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(text) == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Stop any existing scroll */
    tm1637_scroll_stop(handle);

    /* Copy text string */
    handle->scroll_text = strdup(text);
    if (!handle->scroll_text)
    {
        return ESP_ERR_NO_MEM;
    }

    handle->scroll_delay_ms = delay_ms;
    handle->scroll_loop = loop;
    handle->scroll_running = true;

    /* Create scrolling task */
    BaseType_t ret = xTaskCreate(
        scroll_task_function,
        "tm1637_scroll",
        2048,
        handle,
        5,
        &handle->scroll_task);

    if (ret != pdPASS)
    {
        free(handle->scroll_text);
        handle->scroll_text = NULL;
        handle->scroll_running = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tm1637_scroll_stop(tm1637_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->scroll_running)
    {
        handle->scroll_running = false;

        /* Wait for task to finish (max 2 seconds) */
        int wait_count = 0;
        while (handle->scroll_task != NULL && wait_count < 200)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            wait_count++;
        }

        /* Force delete if still running */
        if (handle->scroll_task != NULL)
        {
            vTaskDelete(handle->scroll_task);
            handle->scroll_task = NULL;
        }
    }

    /* Free scroll text memory */
    if (handle->scroll_text)
    {
        free(handle->scroll_text);
        handle->scroll_text = NULL;
    }

    return ESP_OK;
}