#include <math.h>
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
static const char *TAG = "FLIM";

void app_main()
{

    while (1)
    {
        vTaskDelay(1);
    };
}