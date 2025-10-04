#include "bsp_i2c.h"

static const char *TAG = "I2c";

i2c_master_dev_handle_t handle_attitude;
i2c_master_dev_handle_t handle_io;
void bsp_i2c_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = BSP_I2C_NUM,
        .scl_io_num = BSP_I2C_SCL,
        .sda_io_num = BSP_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    ESP_LOGI(TAG, "I2C initialized successfully"); // 输出I2C初始化成功的信息
    // attitude
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMI8658_SENSOR_ADDR,
        .scl_speed_hz = BSP_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &handle_attitude));
    ESP_LOGI(TAG, "I2C attitude initialized successfully"); // 输出I2C初始化成功的信息
    // io
    i2c_device_config_t dev_cfg2 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9557_SENSOR_ADDR,
        .scl_speed_hz = BSP_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg2, &handle_io));
    ESP_LOGI(TAG, "I2C io initialized successfully"); // 输出I2C初始化成功的信息
}
