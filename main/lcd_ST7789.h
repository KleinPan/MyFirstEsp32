#pragma once

#include "bsp_i2c.h"
#include "esp_check.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "driver/spi_master.h"
#include "driver/ledc.h"


// 液晶屏显示

#define LCD_LEDC_CH LEDC_CHANNEL_0

#define BSP_LCD_H_RES (320)
#define BSP_LCD_V_RES (240)
#define BSP_LCD_DRAW_BUFF_SIZE (BSP_LCD_H_RES * BSP_LCD_V_RES)

#define BSP_LCD_SPI_MOSI (GPIO_NUM_40)
#define BSP_LCD_SPI_CLK (GPIO_NUM_41)
#define BSP_LCD_BACKLIGHT (GPIO_NUM_42)
#define BSP_LCD_SPI_CS (GPIO_NUM_NC)
#define BSP_LCD_DC (GPIO_NUM_39)
#define BSP_LCD_RST (GPIO_NUM_NC)

#define BSP_LCD_PIXEL_CLOCK_HZ (80 * 1000 * 1000)
#define LCD_CMD_BITS (8)
#define LCD_PARAM_BITS (8)

// IO扩展芯片
#define LCD_CS_GPIO BIT(0)   // PCA9557_GPIO_NUM_1
#define PA_EN_GPIO BIT(1)    // PCA9557_GPIO_NUM_2
#define DVP_PWDN_GPIO BIT(2) // PCA9557_GPIO_NUM_3
/* LCD display color bits */
#define BSP_LCD_BITS_PER_PIXEL (16)

#define BSP_LCD_SPI_NUM (SPI3_HOST)

extern esp_lcd_panel_handle_t panel_handle;
esp_err_t bsp_display_brightness_init(void);
esp_err_t bsp_display_brightness_set(int brightness_percent);
esp_err_t bsp_display_backlight_on();
esp_err_t bsp_display_backlight_off();

esp_err_t bsp_lcd_init_common(void);
void lcd_set_color(uint16_t color);
void lcd_draw_picture(int x_start, int y_start, int x_end, int y_end, const unsigned char *gImage);
void bsp_lvgl_init(void);

