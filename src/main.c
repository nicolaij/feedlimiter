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

QueueHandle_t xQueueDisplay;
TaskHandle_t xHandleWifi = NULL;
TaskHandle_t xHandleADC = NULL;
TaskHandle_t xHandleConsole = NULL;

void app_main()
{

    init_nvs();
    read_nvs_menu();

    xQueueDisplay = xQueueCreate(1, sizeof(displ_t));

    xTaskCreate(adc_task, "adc_task", 1024 * 8, NULL, configMAX_PRIORITIES - 10, &xHandleADC);
    xTaskCreate(btn_task, "btn_task", 1024 * 4, NULL, configMAX_PRIORITIES - 15, NULL);
    xTaskCreate(console_task, "console_task", 1024 * 4, NULL, configMAX_PRIORITIES - 16, &xHandleConsole);
    xTaskCreate(displ_task, "displ_task", 1024 * 4, NULL, configMAX_PRIORITIES - 17, NULL);

    xTaskCreate(wifi_task, "wifi_task", 1024 * 3, NULL, configMAX_PRIORITIES - 12, &xHandleWifi);

    while (1)
    {
        vTaskDelay(100);
    };
}