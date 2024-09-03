#include <stdio.h>
#include "rc-car.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <u8g2.h>

#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"
#include "icons.h"

static const char *TAG = "u8g2";
static uint8_t queueLength = 16;

#define I2C_HOST  0
#define CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306 true

// The I2C device configuration
#define I2C_HOST			0
#define I2C_SCL_PIN     	GPIO_NUM_48
#define I2C_SDA_PIN     	GPIO_NUM_47
#define I2C_HW_ADDR			0x3C
#define I2C_PIN_NUM_RST		-1

// The LCD panel configuration
#define LCD_PIXEL_CLOCK_HZ  100000
// (400 * 1000)
#define LCD_H_RES			128
#define LCD_V_RES			64
// Bit number used to represent command and parameter
#define LCD_CMD_BITS		8
#define LCD_PARAM_BITS		8

/**
 * @brief Construct a user interface to display data from the hardware
 */
static void task(void *arg) {
    ESP_LOGI(TAG, "Initialize U8G2 HAL");
    // initialize the u8g2 hal
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda = I2C_SDA_PIN;
	u8g2_esp32_hal.scl = I2C_SCL_PIN;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

    ESP_LOGI(TAG, "Initialize U8G2 ssd1306 panel");
	// initialize the u8g2 library
	u8g2_t u8g2;
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
		&u8g2,
		U8G2_R0,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);
	
    ESP_LOGI(TAG, "Set I2C Address");
	// set the display address
	u8x8_SetI2CAddress(&u8g2.u8x8, I2C_HW_ADDR << 1);
	
    ESP_LOGI(TAG, "Initialize Display");
	// initialize the display
	u8g2_InitDisplay(&u8g2);
	
    ESP_LOGI(TAG, "Set Power Saving Mode");
	// wake up the display
	u8g2_SetPowerSave(&u8g2, 0);
	
	// loop
	while(1) {
        ESP_LOGI(TAG, "Graphics loop");

		// draw the hourglass animation, full-half-empty
		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 2, 60, 60, hourglass_full);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		
		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 2, 60, 60, hourglass_half);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		u8g2_ClearBuffer(&u8g2);
		u8g2_DrawXBM(&u8g2, 34, 2, 60, 60, hourglass_empty);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(1000 / portTICK_PERIOD_MS);	
		
		// set font and write hello world
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_timR14_tf);
		u8g2_DrawStr(&u8g2, 2,17,"Hello World!");
		u8g2_SendBuffer(&u8g2);
		
        vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}

QueueHandle_t createUserInterfaceTaskU8G2() {
    QueueHandle_t queue = xQueueCreate(queueLength, sizeof(CommandMessage_t));
    TaskHandle_t userInterfaceTaskHandle = NULL;

    xTaskCreate(
        task, 
        "UserInterfaceTask", 
        USER_INTERFACE_MEMORY, 
        (void *)queue, 
        1, 
        &userInterfaceTaskHandle
    );
    
    return queue;
}
