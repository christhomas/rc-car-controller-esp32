#include <stdio.h>
#include "rc-car.h"
#include "esp_log.h"

void app_main()
{
    printf("RC CAR CONTROLLER\n");
    esp_log_level_set("*", ESP_LOG_DEBUG);

    QueueHandle_t commandQueue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(CommandMessage_t));

    createSnesControllerTask(commandQueue);
    // QueueHandle_t userInterfaceQueue = createUserInterfaceTaskLVGL(commandQueue);
    QueueHandle_t userInterfaceQueue = createUserInterfaceTaskU8G2(commandQueue);
    createBatteryMonitorTask(commandQueue);

    CommandMessage_t message;

    while (true) {
        if (xQueueReceive(commandQueue, &message, portMAX_DELAY)) {
            switch(message.type) {
                case CONTROLLER: 
                    printf("up(%d), down(%d), left(%d), right(%d) || a(%d),  b(%d),  x(%d),  y(%d) || l(%d),  r(%d) || select(%d),  start(%d)\n", 
                        message.controller.up,
                        message.controller.down,
                        message.controller.left,
                        message.controller.right,
                        message.controller.a,
                        message.controller.b,
                        message.controller.x,
                        message.controller.y,
                        message.controller.l,
                        message.controller.r,
                        message.controller.select,
                        message.controller.start
                    );
                    // xQueueSend(userInterfaceQueue, &message, 0);
                break;
                
                case BATTERY:
                    printf("battery voltage: %d, percent %2.2f\n", message.battery.percent, message.battery.voltage);
                break;
            }
        }
    }
}