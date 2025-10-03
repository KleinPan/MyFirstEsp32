#pragma once

#include "esp_err.h"
// https://docs.espressif.com/projects/esp-idf/zh_CN/stable/esp32/api-reference/peripherals/i2c.html
#include "driver/i2c_master.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BSP_I2C_NUM 0
#define BSP_I2C_SDA (GPIO_NUM_1)
#define BSP_I2C_SCL (GPIO_NUM_2)
#define BSP_I2C_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000
#define QMI8658_SENSOR_ADDR 0x6A
#define PCA9557_SENSOR_ADDR 0x19 /*!< Slave address of the MPU9250 sensor */

extern i2c_master_dev_handle_t handle_attitude;
extern i2c_master_dev_handle_t handle_io;
// https://docs.espressif.com/projects/esp-idf/zh_CN/v5.5.1/esp32/migration-guides/release-5.x/5.2/peripherals.html

void bsp_i2c_init(void);
