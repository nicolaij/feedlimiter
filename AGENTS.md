# AGENTS.md

ESP-IDF firmware for a sawmill feed limiter ("Piloramka"/пилорамка): reads motor
current via ADC, drives a reference/feed DAC through a PID loop, exposes a WiFi
settings page + OTA, and can drive a remote 7-segment display over ESP-NOW.
UI, comments, and logs are in Russian.

## Build / flash / monitor

PlatformIO Core is installed but **`pio` is not on PATH**. Use the penv binary or
add it to PATH:

```bash
~/.platformio/penv/bin/pio run -e esp32doit-devkit-v1
~/.platformio/penv/bin/pio run -e c3-display      # ESP32-C3 display-only remote
~/.platformio/penv/bin/pio run -e <env> -t upload
~/.platformio/penv/bin/pio device monitor
```

`pio run` **without `-e` builds both envs** (there is no `default_envs`). Always pass
`-e` to scope a build.

- Platform pinned: `espressif32 @ 6.13.0`; framework ESP-IDF 5.5.3 (`dependencies.lock`).
- `monitor_filters` includes `log2file`, so serial output is saved under `logs/` (gitignored).
- `managed_components/` and `dependencies.lock` are gitignored; the first build after
  a clean checkout fetches components from `components.espressif.com` (needs network).

## Two build targets

Same codebase, split with `-DDISPLAY_ONLY`:

- `esp32doit-devkit-v1` (ESP32): full controller (ADC, DAC, PID, WiFi, ESP-NOW sender).
- `c3-display` (ESP32-C3): remote display only. ADC/DAC code is compiled out and it
  receives `displ_t` frames over ESP-NOW. Uses **different I2C/button pins** than the
  ESP32 build — see the `DISPLAY_ONLY` branch in `include/main.h`.

`displ_t` (in `include/main.h`) is the over-the-wire ESP-NOW frame between the two
firmwares. Changing its layout breaks compatibility between flashed targets.

## Source layout

- `include/main.h` — shared header; declares every task entrypoint and pin/queue externs.
- `src/main.c` — `app_main`; creates all tasks.
- `src/adc.c` — continuous-DMA ADC sampling, PID (`pid_ctrl`), DAC output, the `run_stage`
  state machine, and the display task (TM1637 / HT16K33 auto-detect) plus ESP-NOW.
- `src/network.c` — WiFi SoftAP, HTTP settings page, `/update` OTA, `/ws` WebSocket.
- `src/terminal.c` — serial console menu, NVS-backed settings table, button + LED.
- `components/` — **vendored third-party** ESP-IDF components (`button`, `led_indicator`,
  `ht16k33`, `tm1637`, `pid_ctrl`). Treat as upstream; changes here are usually wrong.
- `src/CMakeLists.txt` uses `FILE(GLOB_RECURSE)`, so new `.c` files are picked up
  automatically — but if a new file is not compiled, force a CMake reconfigure
  (delete `.pio/build/<env>` or `pio run -t clean`) to re-evaluate the glob.

## Settings & NVS

- `menu[]` in `src/terminal.c` is the single source of truth for every tunable
  (Kcalc, Isetmin/max, PID gains, target MAC, ...). Add a setting by adding a
  `menu_t` entry there; read it via `get_menu_val_by_id("id")`.
- Values persist as **float blobs** in NVS namespace `"storage"`, keyed by the `id`
  string. Renaming an `id` orphans the stored value (falls back to the array default).
- Both the web POST handler (`network.c`) and the serial console (`terminal.c`) write
  settings; neither validates better than the `min`/`max` in `menu[]`.

## Gotchas

- `run_stage` (defined in `adc.c`, declared in `main.h`) drives operation: `1` idle/catch
  saw start, `2` stabilize, `3` record idle current, `4` idle/cut transition, `5` cutting
  under PID, `100-111` debug DAC/current overrides, `999` ADC reinit. Remote/debug code
  pokes this variable directly — grep it before touching control flow.
- OTA (`/update`, `update_post_handler`) accepts **either** a firmware image (first byte
  `0xE9`) **or** a SPIFFS image whose size is exactly `0x50000`, dispatched to the next OTA
  partition. That size and the `storage` partition size in `partitions.csv` are coupled.
- ESP-IDF reads the committed per-env config `sdkconfig.<env>` as its live `SDKCONFIG`
  (`sdkconfig.esp32doit-devkit-v1`, `sdkconfig.c3-display`). `sdkconfig.defaults` is applied
  only to options **absent** from the per-env file, so changing it has no effect on values
  already there. To change a setting, edit the per-env `sdkconfig.<env>` (or delete it and
  edit `sdkconfig.defaults` to regenerate).
