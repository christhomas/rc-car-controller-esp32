#include <stdio.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <esp_system.h>

#include "tasks.h"

void joystickTask(void *pvParameters);
void buttonTask(void *pvParameters);

TaskHandle_t taskHandle = NULL;
UBaseType_t taskPriority = 1;
uint32_t taskMemory = 2048;

// Define what buttons to interrupt on
#define NUM_BUTTONS 7
const gpio_num_t BUTTON_PINS[NUM_BUTTONS] = {
    GPIO_NUM_12,
    GPIO_NUM_14,
    GPIO_NUM_27,
    GPIO_NUM_26,
    GPIO_NUM_25,
    GPIO_NUM_33,
    GPIO_NUM_32,
};
QueueHandle_t buttonQueue;

// Function to generate the pin mask
uint64_t generate_pin_mask(const gpio_num_t* pins, int num_pins) {
    uint64_t pin_mask = 0;
    for (int i = 0; i < num_pins; i++) {
        pin_mask |= (1ULL << pins[i]);
    }
    return pin_mask;
}

// Function to create the button queue
QueueHandle_t create_button_queue() {
    return xQueueCreate(10, sizeof(uint32_t));
}

// Interrupt service routine for button press
void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(buttonQueue, &gpio_num, NULL);
}

// Function to configure GPIO pins and attach interrupts
void configure_gpio(const gpio_num_t* pins, int num_pins, uint64_t pin_mask) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = pin_mask,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };

    gpio_config(&io_conf);
    gpio_install_isr_service(0);

    for (int i = 0; i < num_pins; i++) {
        gpio_isr_handler_add(pins[i], button_isr_handler, (void*)pins[i]);
    }
}

void createControllerTask(int core) {
    buttonQueue = create_button_queue();

    // Generate the pin mask for the button pins
    uint64_t pin_mask = generate_pin_mask(BUTTON_PINS, NUM_BUTTONS);
    printf("Pin mask: %llu", pin_mask);

    // Configure GPIO and attach interrupts
    configure_gpio(BUTTON_PINS, NUM_BUTTONS, pin_mask);

    xTaskCreatePinnedToCore(buttonTask, "buttonTask", taskMemory, (void *)buttonQueue, taskPriority, &taskHandle, core);
}

void buttonTask(void *pvParameters) {
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
    uint32_t gpio_num;

    uint32_t index = 0;
    while (true) {
        if (xQueueReceive(queue, &gpio_num, portMAX_DELAY)) {
            printf("[%d] Button pressed on GPIO %d\n", index++, gpio_num);
        }
    }
}  
