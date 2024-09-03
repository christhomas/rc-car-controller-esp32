#include <stdio.h>
#include <stdlib.h>
#include "rc-car.h"

#include <freertos/task.h>
#include <driver/adc.h>
#include <driver/i2c.h>
#include <esp_adc_cal.h>

#define ADC_CHANNEL ADC1_CHANNEL_0
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_12
#define DEFAULT_VREF_MV 1100
#define BATTERY_VOLTS_MIN 3.0
#define BATTERY_VOLTS_MAX 4.2

// Function to read the battery voltage
float read_battery_voltage() {
    // get an average of 64 samples
    int adc_reading = 0;
    const int num_samples = 64;
    for (int i = 0; i < num_samples; i++) {
        adc_reading += adc1_get_raw(ADC_CHANNEL);
    }
    adc_reading /= num_samples;

    // Convert ADC reading to voltage in mV
    esp_adc_cal_characteristics_t *adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF_MV, adc_chars);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    free(adc_chars);

    // Calculate the actual battery voltage using the voltage divider formula
    float r1 = 100, r2 = 100;
    float battery_voltage = (voltage * ((r1 + r1) / r2)) / 1000;
    return battery_voltage;
}

// Function to estimate battery percentage
uint16_t estimate_battery_percentage(float battery_voltage) {
    int battery_percentage = (battery_voltage - BATTERY_VOLTS_MIN) / (BATTERY_VOLTS_MAX - BATTERY_VOLTS_MIN) * 100;
    
    if (battery_percentage > 100) {
        battery_percentage = 100;
    } else if (battery_percentage < 0) {
        battery_percentage = 0;
    }

    return battery_percentage;
}

void init_adc() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);    
}

/**
 * @brief Probe the battery monitor pin for analog value and convert that into a percentage
 */
static void batteryMonitorTask(void *arg) {
    QueueHandle_t queue = (QueueHandle_t)arg;
    CommandMessage_t message = {
        .type = BATTERY,
        .battery.percent = 0,
        .battery.voltage = 0.0f,
    };

    init_adc();

    while (true) {    
        float battery_voltage = read_battery_voltage();
    
        uint16_t battery_percentage = estimate_battery_percentage(battery_voltage);
        message.battery.percent = battery_percentage;
        message.battery.voltage = battery_voltage;

        xQueueSend(queue, &message, 0);

        // printf("Battery Voltage: %.2f V, Battery Level: %d%%\n", battery_voltage, battery_percentage);
            
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void createBatteryMonitorTask(QueueHandle_t queue) {
    TaskHandle_t batteryMonitorTaskHandle = NULL;
    const UBaseType_t batteryMonitorPriority = 1;

    xTaskCreate(
        batteryMonitorTask, 
        "BatteryMonitorTask", 
        8912, 
        (void *)queue, 
        batteryMonitorPriority, 
        &batteryMonitorTaskHandle
    );
}
