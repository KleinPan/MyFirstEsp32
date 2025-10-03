
#include "bsp_i2c.h"

#define PCA9557_INPUT_PORT 0x00
#define PCA9557_OUTPUT_PORT 0x01
#define PCA9557_POLARITY_INVERSION_PORT 0x02
#define PCA9557_CONFIGURATION_PORT 0x03

#define LCD_CS_GPIO BIT(0)   // PCA9557_GPIO_NUM_1
#define PA_EN_GPIO BIT(1)    // PCA9557_GPIO_NUM_2
#define DVP_PWDN_GPIO BIT(2) // PCA9557_GPIO_NUM_3



// https://wiki.lckfb.com/zh-hans/szpi-esp32s3/beginner/audio-output-es8311.html
#define SET_BITS(_m, _s, _v) ((_v) ? (_m) | ((_s)) : (_m) & ~((_s)))

void bsp_expansion_init();

void lcd_cs(uint8_t level); // LCD_CS 控制液晶屏

void pa_en(uint8_t level); // PA_EN 控制音频功放

void dvp_pwdn(uint8_t level); // DVP_PWDN 控制摄像头。

// static uint8_t pca9557_register_read(uint8_t reg_addr);
esp_err_t pca9557_register_write_byte(uint8_t reg_addr, uint8_t data);

esp_err_t pca9557_set_output_state(uint8_t gpio_num, uint8_t level);