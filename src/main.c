#include <math.h>
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <main.h>

static const char *TAG = "FLIM";

void app_main()
{
    xTaskCreate(adc_task, "adc_task", 1024 * 8, NULL, configMAX_PRIORITIES - 10, NULL);
    xTaskCreate(btn_task, "btn_task", 1024 * 4, NULL, configMAX_PRIORITIES - 15, NULL);
    xTaskCreate(console_task, "console_task", 1024 * 4, NULL, configMAX_PRIORITIES - 15, NULL);


    while (1)
    {
        vTaskDelay(1);
    };
}