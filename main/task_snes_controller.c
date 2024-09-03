#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "rc-car.h"

#define LATCH_PIN GPIO_NUM_4
#define CLOCK_PIN GPIO_NUM_5
#define DATA_PIN  GPIO_NUM_6

// SNES button mappings
#define BUTTON_B        0x01    // 0b0000000000000000 0000000000000001
#define BUTTON_Y        0x02    // 0b0000000000000000 0000000000000010
#define BUTTON_SELECT   0x04    // 0b0000000000000000 0000000000000100
#define BUTTON_START    0x08    // 0b0000000000000000 0000000000001000
#define BUTTON_UP       0x10    // 0b0000000000000000 0000000000010000
#define BUTTON_DOWN     0x20    // 0b0000000000000000 0000000000100000
#define BUTTON_LEFT     0x40    // 0b0000000000000000 0000000001000000
#define BUTTON_RIGHT    0x80    // 0b0000000000000000 0000000010000000
#define BUTTON_A        0x100   // 0b0000000000000000 0000000100000000
#define BUTTON_X        0x200   // 0b0000000000000000 0000001000000000
#define BUTTON_L        0x400   // 0b0000000000000000 0000010000000000
#define BUTTON_R        0x800   // 0b0000000000000000 0000100000000000

#define wait(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define LATCH_WAIT 12
#define CLOCK_WAIT 6

void init_button_controller() {
    gpio_set_direction(CLOCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LATCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DATA_PIN, GPIO_MODE_INPUT);
    
    // gpio_pullup_en(DATA_PIN);
}

// Latch the current button states into the shift register
void latch () {
    gpio_set_level(LATCH_PIN, 1);
    wait(LATCH_WAIT);
    
    gpio_set_level(LATCH_PIN, 0);
}

// Pulse the clock to shift the next bit
uint32_t clock () {
    wait(CLOCK_WAIT);
    uint32_t bit = gpio_get_level(DATA_PIN);

    gpio_set_level(CLOCK_PIN, 1);
    wait(CLOCK_WAIT);
    
    gpio_set_level(CLOCK_PIN, 0);
    wait(CLOCK_WAIT);

    return bit;
}

uint16_t read_button_controller() {
    uint16_t buttons = 0;
    
    latch();
    
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t bit = clock();

        buttons |= bit << i; 
    }
    
    return ~buttons;
}

void snesControllerTask(void *arg) {
    init_button_controller();

    QueueHandle_t queue = (QueueHandle_t) arg;
    uint16_t buttons = 0;
    CommandMessage_t message;
    bool hasButton = false;

    while (true) {
        TickType_t start_time = xTaskGetTickCount();

        memset(&message, 0, sizeof(message));
        message.type = CONTROLLER;

        buttons = read_button_controller();
        hasButton = false;

        if (buttons == 0) continue;

        if (buttons & BUTTON_B) {
            message.controller.b = true;
            hasButton = true;
        }

        if (buttons & BUTTON_Y)  {
            message.controller.y = true;
            hasButton = true;
        }

        if (buttons & BUTTON_SELECT)  {
            message.controller.select = true;
            hasButton = true;
        }

        if (buttons & BUTTON_START)  {
            message.controller.start = true;
            hasButton = true;
        }

        if (buttons & BUTTON_UP)  {
            message.controller.up = true;
            hasButton = true;
        }

        if (buttons & BUTTON_DOWN)  {
            message.controller.down = true;
            hasButton = true;
        }

        if (buttons & BUTTON_LEFT)  {
            message.controller.left = true;
            hasButton = true;
        }

        if (buttons & BUTTON_RIGHT)  {
            message.controller.right = true;
            hasButton = true;
        }

        if (buttons & BUTTON_A)  {
            message.controller.a = true;
            hasButton = true;
        }

        if (buttons & BUTTON_X)  {
            message.controller.x = true;
            hasButton = true;
        }

        if (buttons & BUTTON_L)  {
            message.controller.l = true;
            hasButton = true;
        }

        if (buttons & BUTTON_R)  {
            message.controller.r = true;
            hasButton = true;
        }

        // If any button was set, then send this to be processed
        if (hasButton) {
            xQueueSend(queue, (void *)&message, 0);
        }

        TickType_t end_time = xTaskGetTickCount();
        uint32_t elapsed_ms = (end_time - start_time) * portTICK_PERIOD_MS;
        uint32_t wait_ms = SNES_CONTROLLER_INTERVAL - elapsed_ms;

        if (wait_ms > 0) {
            wait(wait_ms);
        }        
    }
}

void createSnesControllerTask(QueueHandle_t commandQueue) {
    TaskHandle_t snesControllerTaskHandle = NULL;

    xTaskCreate(
        snesControllerTask,
        "SnesControllerTask",
        4096,
        (void *)commandQueue,
        1,
        &snesControllerTaskHandle
    );
}