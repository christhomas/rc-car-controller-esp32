#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks.h"

void app_main()
{
    createControllerTask(0);
    
    vTaskDelete(NULL);
}