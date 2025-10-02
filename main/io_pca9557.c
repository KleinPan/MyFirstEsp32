#include "io_pca9557.h"

// 初始化PCA9557 IO扩展芯片
void pca9557_init()
{
    // 写入输出引脚默认值 DVP_PWDN=1  PA_EN = 0  LCD_CS = 1
    pca9557_register_write_byte(PCA9557_OUTPUT_PORT, 0x05);
    // 配置控制脚，寄存器3
    pca9557_register_write_byte(PCA9557_CONFIGURATION_PORT, 0xf8);
}

static uint8_t pca9557_register_read(uint8_t reg_addr)
{
    uint8_t data;
    i2c_master_write_read_device(BSP_I2C_NUM, PCA9557_SENSOR_ADDR, &reg_addr, 1, &data, 1, 1000 / portTICK_PERIOD_MS);
    return data;
}

// 给PCA9557芯片的寄存器写值
esp_err_t pca9557_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(BSP_I2C_NUM, PCA9557_SENSOR_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);

    return ret;
}

// 控制 PCA9557_LCD_CS 引脚输出高低电平 参数0输出低电平 参数1输出高电平
void lcd_cs(uint8_t level)
{
    pca9557_set_output_state(LCD_CS_GPIO, level);
}

// 控制 PCA9557_PA_EN 引脚输出高低电平 参数0输出低电平 参数1输出高电平
void pa_en(uint8_t level)
{
    pca9557_set_output_state(PA_EN_GPIO, level);
}

// 控制 PCA9557_DVP_PWDN 引脚输出高低电平 参数0输出低电平 参数1输出高电平
void dvp_pwdn(uint8_t level)
{
    pca9557_set_output_state(DVP_PWDN_GPIO, level);
}

esp_err_t pca9557_set_output_state(uint8_t gpio_num, uint8_t level)
{
    uint8_t reg_val = pca9557_register_read(PCA9557_OUTPUT_PORT);
    reg_val = SET_BITS(reg_val, gpio_num, level);
    return pca9557_register_write_byte(PCA9557_OUTPUT_PORT, reg_val);
}