# TM1637 7-Segment Display Driver for ESP-IDF

A comprehensive ESP-IDF component for controlling TM1637-based 7-segment LED displays. This driver provides full control over brightness, display content, and includes advanced features like text scrolling.

## Features

- ✅ **Full TM1637 Protocol Implementation** - Based on official datasheet
- ✅ **Brightness Control** - 8 levels (0-7) with on/off control
- ✅ **Decimal & Hexadecimal Numbers** - Display integers in multiple formats
- ✅ **Alphanumeric Support** - Display text with A-Z, 0-9, and symbols
- ✅ **Text Scrolling** - Blocking and non-blocking scrolling animations
- ✅ **Thread-Safe** - Mutex-protected operations for multi-task safety
- ✅ **Low-Level Control** - Direct segment manipulation available
- ✅ **Easy Integration** - Simple API with comprehensive examples

## Hardware Connection

| TM1637 Pin | ESP32 Pin | Description  |
| ---------- | --------- | ------------ |
| VCC        | 3.3V/5V   | Power supply |
| GND        | GND       | Ground       |
| CLK        | Any GPIO  | Clock signal |
| DIO        | Any GPIO  | Data I/O     |

**Note:** External pull-up resistors (4.7kΩ - 10kΩ) are recommended on CLK and DIO pins for reliable operation.

## Installation

### Using ESP Component Registry (Recommended)

```bash
idf.py add-dependency "heronet/tm1637^1.0.0"
```

### Manual Installation

Clone this repository into your project's `components` directory:

```bash
cd your_project/components
git clone https://github.com/heronet/esp-idf-tm1637.git tm1637
```

## Quick Start

```c
#include "tm1637.h"

void app_main(void)
{
    // Configure pins
    tm1637_config_t config = {
        .clk_pin = GPIO_NUM_18,
        .dio_pin = GPIO_NUM_19,
        .bit_delay_us = 100
    };

    // Initialize display
    tm1637_handle_t display;
    tm1637_init(&config, &display);

    // Set brightness (0-7)
    tm1637_set_brightness(display, 5, true);

    // Display a number
    tm1637_show_number(display, 1234, false, 4, 0);

    // Display text
    tm1637_write_string(display, "HI");

    // Scroll text
    tm1637_scroll_text(display, "HELLO WORLD", 300);

    // Clean up
    tm1637_deinit(display);
}
```

## API Reference

### Initialization

#### `tm1637_init()`

Initialize the TM1637 display.

```c
esp_err_t tm1637_init(const tm1637_config_t *config, tm1637_handle_t *handle);
```

**Parameters:**

- `config`: Configuration structure containing GPIO pins
- `handle`: Output handle for the device

**Returns:** `ESP_OK` on success

---

#### `tm1637_deinit()`

Deinitialize and free resources.

```c
esp_err_t tm1637_deinit(tm1637_handle_t handle);
```

### Display Control

#### `tm1637_set_brightness()`

Set display brightness and on/off state.

```c
esp_err_t tm1637_set_brightness(tm1637_handle_t handle, uint8_t brightness, bool on);
```

**Parameters:**

- `brightness`: Level 0-7 (0=dimmest, 7=brightest)
- `on`: true to turn on, false to turn off

**Example:**

```c
tm1637_set_brightness(display, 7, true);   // Maximum brightness, ON
tm1637_set_brightness(display, 0, false);  // Turn off display
```

---

#### `tm1637_display_on()` / `tm1637_display_off()`

Convenience functions to turn display on/off.

```c
esp_err_t tm1637_display_on(tm1637_handle_t handle);
esp_err_t tm1637_display_off(tm1637_handle_t handle);
```

---

#### `tm1637_clear()`

Clear the entire display.

```c
esp_err_t tm1637_clear(tm1637_handle_t handle);
```

### Number Display

#### `tm1637_show_number()`

Display a decimal number (supports negative numbers).

```c
esp_err_t tm1637_show_number(tm1637_handle_t handle, int32_t number,
                             bool leading_zeros, uint8_t length, uint8_t position);
```

**Parameters:**

- `number`: Number to display (-9999 to 9999 for 4-digit display)
- `leading_zeros`: Show leading zeros if true
- `length`: Number of digits to use
- `position`: Starting position (0-3)

**Examples:**

```c
tm1637_show_number(display, 42, false, 4, 0);    // "  42"
tm1637_show_number(display, 42, true, 4, 0);     // "0042"
tm1637_show_number(display, -123, false, 4, 0);  // "-123"
```

---

#### `tm1637_show_number_hex()`

Display a hexadecimal number.

```c
esp_err_t tm1637_show_number_hex(tm1637_handle_t handle, uint32_t number,
                                 bool leading_zeros, uint8_t length, uint8_t position);
```

**Examples:**

```c
tm1637_show_number_hex(display, 0xABCD, true, 4, 0);  // "ABCD"
tm1637_show_number_hex(display, 0xFF, false, 4, 0);   // "  FF"
```

### Text Display

#### `tm1637_write_string()`

Display alphanumeric text.

```c
esp_err_t tm1637_write_string(tm1637_handle_t handle, const char *text);
```

**Supported Characters:**

- Digits: `0-9`
- Letters: `A-Z, a-z` (limited to 7-segment representation)
- Symbols: `-` (minus), ` ` (space)
- Decimal point: `.` (use after a character, e.g., "12.5")

**Examples:**

```c
tm1637_write_string(display, "HELP");     // Display "HELP"
tm1637_write_string(display, "12.5");     // Display "12.5" with decimal point
tm1637_write_string(display, "A-bc");     // Display "A-bc"
```

### Text Scrolling

#### `tm1637_scroll_text()`

Scroll text across the display (blocking).

```c
esp_err_t tm1637_scroll_text(tm1637_handle_t handle, const char *text, uint32_t delay_ms);
```

**Parameters:**

- `text`: Text string to scroll
- `delay_ms`: Delay between scroll steps (recommended: 200-500ms)

**Example:**

```c
tm1637_scroll_text(display, "HELLO WORLD", 300);  // Blocks until done
```

---

#### `tm1637_scroll_start()`

Start scrolling text in the background (non-blocking).

```c
esp_err_t tm1637_scroll_start(tm1637_handle_t handle, const char *text,
                              uint32_t delay_ms, bool loop);
```

**Parameters:**

- `text`: Text string to scroll (copied internally)
- `delay_ms`: Delay between scroll steps
- `loop`: If true, scroll continuously; if false, scroll once

**Example:**

```c
// Start continuous scrolling
tm1637_scroll_start(display, "TEMPERATURE 25C", 250, true);

// Do other work...

// Stop scrolling later
tm1637_scroll_stop(display);
```

---

#### `tm1637_scroll_stop()`

Stop background scrolling.

```c
esp_err_t tm1637_scroll_stop(tm1637_handle_t handle);
```

### Low-Level Control

#### `tm1637_set_segments()`

Directly set raw segment data for precise control.

```c
esp_err_t tm1637_set_segments(tm1637_handle_t handle, const uint8_t *segments,
                              uint8_t length, uint8_t position);
```

**Segment Definitions:**

- `TM1637_SEG_A` - Top segment
- `TM1637_SEG_B` - Top-right segment
- `TM1637_SEG_C` - Bottom-right segment
- `TM1637_SEG_D` - Bottom segment
- `TM1637_SEG_E` - Bottom-left segment
- `TM1637_SEG_F` - Top-left segment
- `TM1637_SEG_G` - Middle segment
- `TM1637_SEG_DP` - Decimal point

**Example:**

```c
uint8_t custom[] = {
    TM1637_SEG_A | TM1637_SEG_B | TM1637_SEG_C,  // Custom pattern
    TM1637_SEG_G | TM1637_SEG_DP,                // Minus with dot
};
tm1637_set_segments(display, custom, 2, 0);
```

---

#### `tm1637_encode_digit()`

Encode a digit (0-15) to 7-segment pattern.

```c
uint8_t tm1637_encode_digit(uint8_t digit);
```

## Examples

### Basic Number Display

```c
#include "tm1637.h"

void app_main(void) {
    tm1637_config_t config = {
        .clk_pin = GPIO_NUM_18,
        .dio_pin = GPIO_NUM_19,
        .bit_delay_us = 100
    };

    tm1637_handle_t display;
    tm1637_init(&config, &display);
    tm1637_set_brightness(display, 5, true);

    int count = 0;
    while (1) {
        tm1637_show_number(display, count++, false, 4, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### Temperature Display

```c
void display_temperature(tm1637_handle_t display, float temp) {
    // Convert to integer (e.g., 25.6 -> 256)
    int temp_int = (int)(temp * 10);

    // Display with decimal point
    char temp_str[8];
    snprintf(temp_str, sizeof(temp_str), "%d.%d", temp_int / 10, temp_int % 10);
    tm1637_write_string(display, temp_str);
}
```

### Clock Display

```c
void display_time(tm1637_handle_t display, int hour, int minute) {
    // Format: HH:MM (use colon segment if available)
    uint8_t time_display[4];
    time_display[0] = tm1637_encode_digit(hour / 10);
    time_display[1] = tm1637_encode_digit(hour % 10) | TM1637_SEG_DP;
    time_display[2] = tm1637_encode_digit(minute / 10);
    time_display[3] = tm1637_encode_digit(minute % 10);

    tm1637_set_segments(display, time_display, 4, 0);
}
```

## Folder Structure

```
tm1637/
├── CMakeLists.txt
├── idf_component.yml
├── LICENSE
├── README.md
├── include/
│   └── tm1637.h
├── src/
│   └── tm1637.c
└── examples/
    ├── basic_display/
    └── scrolling_text/
```

## License

Apache License 2.0 - See LICENSE file for details

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Credits

Created by Siratul Islam  
Based on TM1637 datasheet specifications

## Support

For issues, questions, or suggestions, please open an issue on GitHub.
