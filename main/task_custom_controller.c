#include <stdio.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include <esp_system.h>

#include "rc-car.h"

TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t joystickTaskHandle = NULL;

#define NUM_BUTTONS 5
const gpio_num_t BUTTON_PINS[NUM_BUTTONS] = {
    GPIO_NUM_35,
    GPIO_NUM_4,
    GPIO_NUM_16,
    GPIO_NUM_17,
    GPIO_NUM_5,
};

typedef struct {
    uint32_t gpio_num;
    int current_state;
    int previous_state;
} button_state_t;

button_state_t button_states[NUM_BUTTONS];
QueueHandle_t buttonQueue;
uint32_t buttonQueueLength = 20;

const enum AXIS {
    X_AXIS = 0,
    Y_AXIS = 1,
};

#define NUM_AXIS 2
const gpio_num_t JOYSTICK_PINS[NUM_AXIS] = {
    ADC1_CHANNEL_4,
    ADC1_CHANNEL_5,
};

// Function to initialize button states
void init_button_states() {
    for (int i = 0; i < NUM_BUTTONS; i++) {
        button_states[i].gpio_num = BUTTON_PINS[i];
        button_states[i].current_state = gpio_get_level(BUTTON_PINS[i]);
        button_states[i].previous_state = button_states[i].current_state;
    }
}

// Function to generate the pin mask
uint64_t generate_pin_mask(const gpio_num_t* pins, int num_pins) {
    uint64_t pin_mask = 0;
    for (int i = 0; i < num_pins; i++) {
        pin_mask |= (1ULL << pins[i]);
    }
    return pin_mask;
}

QueueHandle_t create_button_queue() {
    return xQueueCreate(buttonQueueLength, sizeof(uint32_t));
}

void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(buttonQueue, &gpio_num, NULL);
}

void configure_gpio(const gpio_num_t* pins, int num_pins, uint64_t pin_mask) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
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

void buttonTask(void *pvParameters) {
    buttonQueue = create_button_queue();

    uint32_t gpio_num;
    uint32_t seq = 0;

    while (true) {
        if (xQueueReceive(buttonQueue, &gpio_num, portMAX_DELAY)) {
            for (int i = 0; i < NUM_BUTTONS; i++) {
                // Only process buttons that are pressed
                if (button_states[i].gpio_num == gpio_num) {
                    button_states[i].previous_state = button_states[i].current_state;
                    button_states[i].current_state = gpio_get_level(button_states[i].gpio_num);

                    if (button_states[i].current_state != button_states[i].previous_state) {
                        printf("[%lu]: Button on GPIO %lu changed to %d\n", seq++, button_states[i].gpio_num, button_states[i].current_state);
                    }
                }
            }
        }
    }
}

void joystickTask(void *pvParameters) {
    int vrx = 0;
    int vry = 0;
    int wobble = 5;
    bool update = false;
    int milliseconds = 100;

    while (true) {
        // Read the analog values from VRx and VRy
        int vrx_value = adc1_get_raw(JOYSTICK_PINS[X_AXIS]);
        int vry_value = adc1_get_raw(JOYSTICK_PINS[Y_AXIS]);

        // Normalize the values to a range (e.g., 0-100)
        int vrx_new = vrx_value * 100 / 4095;
        int vry_new = vry_value * 100 / 4095;

        if (vrx_new != vrx || vry_new != vry) {
            update = false;

            // Ignore minor tiny wobbles to remove noisy inputs
            if(abs(vrx_new - vrx) > wobble) update = true;
            if(abs(vry_new - vry) > wobble) update = true;

            if (update) {
                vrx = vrx_new;
                vry = vry_new;

                // Print the normalized values
                printf("VRx: %d, VRy: %d\n", vrx, vry);
            }
        }

        // Add a delay to avoid flooding the output
        vTaskDelay(pdMS_TO_TICKS(milliseconds));
    }
}

void createCustomControllerTask(QueueHandle_t commandQueue) {
    // Generate the pin mask for the button pins
    uint64_t pin_mask = generate_pin_mask(BUTTON_PINS, NUM_BUTTONS);

    // Initialize button states
    init_button_states();

    // Configure GPIO and attach interrupts
    configure_gpio(BUTTON_PINS, NUM_BUTTONS, pin_mask);

    xTaskCreate(buttonTask, "buttonTask", CUSTOM_CONTROLLER_MEMORY, NULL, 1, &buttonTaskHandle);
    xTaskCreate(joystickTask, "joystickTask", CUSTOM_CONTROLLER_MEMORY, NULL, 1, &joystickTaskHandle);
}
