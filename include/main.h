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

#define LED_PIN GPIO_NUM_19
#define BTN_PIN GPIO_NUM_39

typedef struct
{
    const char id[10];
    const char name[64];
    const char izm[8];
    int val;
    const int min;
    const int max;
} menu_t;

void btn_task(void *arg);
void console_task(void *arg);
void adc_task(void *arg);

#endif
