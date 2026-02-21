#ifndef MAIN_H_
#define MAIN_H_

#include "esp_idf_version.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "freertos/ringbuf.h"

#include <sys/time.h>
#include "esp_sleep.h"

#include "esp_now.h"
#include "esp_mac.h"

#if !defined DISPLAY_ONLY
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define BTN_PIN GPIO_NUM_0
#else
#define SDA_PIN GPIO_NUM_19
#define SCL_PIN GPIO_NUM_18
#define BTN_PIN GPIO_NUM_9

#endif

#define LED_PIN GPIO_NUM_2
#define FEED_FORWARD_PIN GPIO_NUM_4
#define DISABLE_PIN GPIO_NUM_15
#define ADC_CHANNEL_CURRENT ADC_CHANNEL_5
#define ADC_CHANNEL_SET ADC_CHANNEL_0

typedef struct
{
    const char id[10];
    const char name[64];
    const char izm[8];
    float val;
    const float min;
    const float max;
} menu_t;

typedef struct
{
    int curr;
    int set;
    bool dots;
} displ_t;

extern TaskHandle_t xHandleWifi;
extern TaskHandle_t xHandleADC;
extern TaskHandle_t xHandleConsole;

extern int run_stage;

#define NOTYFY_WIFI BIT0
#define NOTYFY_WIFI_SWITCH BIT10
#define NOTYFY_WIFI_STOP BIT1
#define NOTYFY_WIFI_ESPNOW BIT2
#define REBOOT_NOW BIT3

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

esp_err_t init_nvs();
esp_err_t read_nvs_menu();

float get_menu_val_by_id(const char *id);
esp_err_t set_menu_val_by_id(const char *id, float value);

void btn_task(void *arg);
void console_task(void *arg);
void adc_task(void *arg);
void displ_task(void *arg);
void wifi_task(void *arg);

int get_menu_html(char *buf);

esp_err_t ws_send_data(const char *str, const int len);

#endif
