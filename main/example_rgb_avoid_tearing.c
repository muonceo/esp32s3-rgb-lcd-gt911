/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_gt911.h"
#include "lv_demos.h"
#include "lvgl_port.h"
#include "esp_lcd_panel_rgb.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_H_RES               (800)
#define EXAMPLE_LCD_V_RES               (480)
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ      (18 * 1000 * 1000)
#define EXAMPLE_RGB_BIT_PER_PIXEL       (16)
#define EXAMPLE_RGB_DATA_WIDTH          (16)
#define EXAMPLE_RGB_BOUNCE_BUFFER_SIZE  (EXAMPLE_LCD_H_RES * CONFIG_EXAMPLE_LCD_RGB_BOUNCE_BUFFER_HEIGHT)
#define EXAMPLE_LCD_IO_RGB_DISP         (-1)             // -1 if not used
#define EXAMPLE_LCD_IO_RGB_VSYNC        (GPIO_NUM_40)
#define EXAMPLE_LCD_IO_RGB_HSYNC        (GPIO_NUM_39)
#define EXAMPLE_LCD_IO_RGB_DE           (GPIO_NUM_41)
#define EXAMPLE_LCD_IO_RGB_PCLK         (GPIO_NUM_42)
#define EXAMPLE_LCD_IO_RGB_DATA0        (GPIO_NUM_15)
#define EXAMPLE_LCD_IO_RGB_DATA1        (GPIO_NUM_7)
#define EXAMPLE_LCD_IO_RGB_DATA2        (GPIO_NUM_6)
#define EXAMPLE_LCD_IO_RGB_DATA3        (GPIO_NUM_5)
#define EXAMPLE_LCD_IO_RGB_DATA4        (GPIO_NUM_4)
#define EXAMPLE_LCD_IO_RGB_DATA5        (GPIO_NUM_9)
#define EXAMPLE_LCD_IO_RGB_DATA6        (GPIO_NUM_46)
#define EXAMPLE_LCD_IO_RGB_DATA7        (GPIO_NUM_3)
#define EXAMPLE_LCD_IO_RGB_DATA8        (GPIO_NUM_8)
#define EXAMPLE_LCD_IO_RGB_DATA9        (GPIO_NUM_16)
#define EXAMPLE_LCD_IO_RGB_DATA10       (GPIO_NUM_1)
#define EXAMPLE_LCD_IO_RGB_DATA11       (GPIO_NUM_14)
#define EXAMPLE_LCD_IO_RGB_DATA12       (GPIO_NUM_21)
#define EXAMPLE_LCD_IO_RGB_DATA13       (GPIO_NUM_47)
#define EXAMPLE_LCD_IO_RGB_DATA14       (GPIO_NUM_48)
#define EXAMPLE_LCD_IO_RGB_DATA15       (GPIO_NUM_45)
// #define EXAMPLE_LCD_IO_SPI_CS           (GPIO_NUM_39)
// #define EXAMPLE_LCD_IO_SPI_SCL          (GPIO_NUM_48)
// #define EXAMPLE_LCD_IO_SPI_SDA          (GPIO_NUM_47)
#define EXAMPLE_LCD_IO_RST              (-1)             // -1 if not used
#define EXAMPLE_PIN_NUM_BK_LIGHT        (GPIO_NUM_2)    // -1 if not used
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL   (1)
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL  !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your touch spec ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911
#define TOUCH_HOST                      (I2C_NUM_0)
#define EXAMPLE_PIN_NUM_TOUCH_SCL       (GPIO_NUM_20)
#define EXAMPLE_PIN_NUM_TOUCH_SDA       (GPIO_NUM_19)
#define EXAMPLE_PIN_NUM_TOUCH_RST       (-1)            // -1 if not used
#define EXAMPLE_PIN_NUM_TOUCH_INT       (-1)            // -1 if not used
#endif

static const char *TAG = "example";

IRAM_ATTR static bool rgb_lcd_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    return lvgl_port_notify_rgb_vsync();
}

void app_main()
{
    if (EXAMPLE_PIN_NUM_BK_LIGHT >= 0) {
        ESP_LOGI(TAG, "Turn off LCD backlight");
        gpio_config_t bk_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
        };
        ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    }

    ESP_LOGI(TAG, "Install rgb panel driver");
    esp_lcd_panel_handle_t lcd_handle = NULL;
    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .psram_trans_align = 64,
        .data_width = EXAMPLE_RGB_DATA_WIDTH,
        .bits_per_pixel = EXAMPLE_RGB_BIT_PER_PIXEL,
        .de_gpio_num = EXAMPLE_LCD_IO_RGB_DE,
        .pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK,
        .vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC,
        .hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC,
        .disp_gpio_num = EXAMPLE_LCD_IO_RGB_DISP,
        .data_gpio_nums = {
            EXAMPLE_LCD_IO_RGB_DATA0,
            EXAMPLE_LCD_IO_RGB_DATA1,
            EXAMPLE_LCD_IO_RGB_DATA2,
            EXAMPLE_LCD_IO_RGB_DATA3,
            EXAMPLE_LCD_IO_RGB_DATA4,
            EXAMPLE_LCD_IO_RGB_DATA5,
            EXAMPLE_LCD_IO_RGB_DATA6,
            EXAMPLE_LCD_IO_RGB_DATA7,
            EXAMPLE_LCD_IO_RGB_DATA8,
            EXAMPLE_LCD_IO_RGB_DATA9,
            EXAMPLE_LCD_IO_RGB_DATA10,
            EXAMPLE_LCD_IO_RGB_DATA11,
            EXAMPLE_LCD_IO_RGB_DATA12,
            EXAMPLE_LCD_IO_RGB_DATA13,
            EXAMPLE_LCD_IO_RGB_DATA14,
            EXAMPLE_LCD_IO_RGB_DATA15,
        },
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            // The following parameters should refer to LCD spec
            .hsync_back_porch = 16,
            .hsync_front_porch = 210,
            .hsync_pulse_width = 30,
            .vsync_back_porch = 10,
            .vsync_front_porch = 22,
            .vsync_pulse_width = 13,
            .flags.pclk_active_neg = true,
        },
        .flags.fb_in_psram = 1,
        .num_fbs = LVGL_PORT_LCD_RGB_BUFFER_NUMS,
        .bounce_buffer_size_px = EXAMPLE_RGB_BOUNCE_BUFFER_SIZE,
    };

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&rgb_config, &lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handle));
    esp_lcd_panel_disp_on_off(lcd_handle, true);

    esp_lcd_touch_handle_t tp_handle = NULL;
#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911
    ESP_LOGI(TAG, "Initialize I2C bus");
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_TOUCH_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = EXAMPLE_PIN_NUM_TOUCH_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400 * 1000,
    };
    ESP_ERROR_CHECK(i2c_param_config(TOUCH_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_HOST, i2c_conf.mode, 0, 0, 0));

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

    ESP_LOGI(TAG, "Initialize I2C panel IO");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle));

    ESP_LOGI(TAG, "Initialize touch controller GT911");
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST,
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp_handle));
#endif // CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911

    ESP_ERROR_CHECK(lvgl_port_init(lcd_handle, tp_handle));

    esp_lcd_rgb_panel_event_callbacks_t cbs = {
#if EXAMPLE_RGB_BOUNCE_BUFFER_SIZE > 0
        .on_bounce_frame_finish = rgb_lcd_on_vsync_event,
#else
        .on_vsync = rgb_lcd_on_vsync_event,
#endif
    };
    esp_lcd_rgb_panel_register_event_callbacks(lcd_handle, &cbs, NULL);

    if (EXAMPLE_PIN_NUM_BK_LIGHT >= 0) {
        ESP_LOGI(TAG, "Turn on LCD backlight");
        gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
    }

    ESP_LOGI(TAG, "Display LVGL demos");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(-1)) {
        // lv_demo_stress();
        // lv_demo_benchmark();
        lv_demo_music();
        // lv_demo_widgets();

        // Release the mutex
        lvgl_port_unlock();
    }
}
