#include <stdio.h>
#include <string.h>
#include "rc-car.h"
#include <driver/gpio.h>
// #include <driver/spi_master.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_err.h>

#include "sdkconfig.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_vendor.h"
#include <lvgl.h>

static const char *TAG = "lvgl_ui";
static uint8_t queueLength = 16;

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

// /* The LVGL port component calls esp_lcd_panel_draw_bitmap API for send data to the screen. There must be called
// lvgl_port_flush_ready(disp) after each transaction to display. The best way is to use on_color_trans_done
// callback from esp_lcd IO config structure. In IDF 5.1 and higher, it is solved inside LVGL port component. */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t * disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}

static void init_i2c () {
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, i2c_conf.mode, 0, 0, 0));
}

static void init_esp_panel (esp_lcd_panel_io_handle_t *io_handle, esp_lcd_panel_handle_t *panel_handle) {
    ESP_LOGI(TAG, "Install panel IO");

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .control_phase_bytes = 1,       // According to SSD1306 datasheet
        .lcd_cmd_bits = LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,             // According to SSD1306 datasheet
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_i2c(
            (esp_lcd_i2c_bus_handle_t)I2C_HOST, 
            &io_config, 
            io_handle
        )
    );

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = I2C_PIN_NUM_RST,
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_ssd1306(
            *io_handle, 
            &panel_config, 
            panel_handle
        )
    );

    ESP_ERROR_CHECK(esp_lcd_panel_reset(*panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(*panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(*panel_handle, true));
}

static void init_lvgl (
    esp_lcd_panel_io_handle_t io_handle, 
    esp_lcd_panel_handle_t panel_handle, 
    lv_disp_t **disp
) {
    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * LCD_V_RES,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    
    *disp = lvgl_port_add_disp(&disp_cfg);
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, *disp);

    /* Rotation of the screen */
    lv_disp_set_rotation(*disp, LV_DISP_ROT_180);
}

/**
 * @brief Construct a user interface to display data from the hardware
 */
static void task(void *arg) {
    QueueHandle_t queue = (QueueHandle_t) arg;

    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;
    lv_disp_t *disp = NULL;

    init_i2c();
    init_esp_panel(&io_handle, &panel_handle);
    init_lvgl(io_handle, panel_handle, &disp);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    
    lv_obj_t *helloWorldLabel = lv_label_create(scr);
    lv_obj_set_width(helloWorldLabel, disp->driver->hor_res);
    lv_obj_align(helloWorldLabel, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *counterLabel = lv_label_create(scr);
    lv_obj_set_width(counterLabel, disp->driver->hor_res);
    lv_obj_align(counterLabel, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    
    char counterText[32];
    uint32_t counter = 0;

    CommandMessage_t *message = (CommandMessage_t *)malloc(sizeof(CommandMessage_t));

   while (true) {
        // if (xQueueReceive(queue, message, portMAX_DELAY)) {
        //     switch(message->type) {
        //         case MOTOR:
        //             // printf("We received a motor message, speed = '%d'\n", message->motor.speed);
        //             snprintf(motorText, 32, "S: %d", message->motor.speed);
        //             lv_label_set_text(motorLabel, motorText);
        //             break;

        //         case BATTERY:
        //             // printf("We received a battery message, voltage = '%2.2f', percent = '%d'\n", message->battery.voltage, message->battery.percent);
        //             snprintf(batteryText, 32, "P: %d%% (%2.2f v)", message->battery.percent, message->battery.voltage);
        //             lv_label_set_text(batteryLabel, batteryText);
        //             break;
        //     }
        // }

        snprintf(counterText, 10, "C: %lu", counter++);
        lv_label_set_text(counterLabel, counterText);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

QueueHandle_t createUserInterfaceTaskLVGL() {
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
