#pragma once

#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define BSP_I2C_NUM     0
#define BSP_I2C_SDA     (GPIO_NUM_1)
#define BSP_I2C_SCL     (GPIO_NUM_2)
#define BSP_I2C_FREQ_HZ 400000

//static const char *TAG = "bsp_i2c";

//https://docs.espressif.com/projects/esp-idf/zh_CN/v5.5.1/esp32/migration-guides/release-5.x/5.2/peripherals.html

esp_err_t bsp_i2c_init(void);
