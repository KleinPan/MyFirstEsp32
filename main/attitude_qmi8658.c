
#include "attitude_qmi8658.h"
#include <math.h>

static const char *TAG = "qmi8658";

static esp_err_t qmi8658_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(handle_attitude, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t qmi8658_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buff[2] = {reg_addr, data};
    return i2c_master_transmit(handle_attitude, write_buff, sizeof(write_buff), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void bsp_attitude_init(void)
{
    uint8_t id;

    qmi8658_register_read(0x00, &id, 1);
    while (id != 0x05)
    {
        vTaskDelay(I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        qmi8658_register_read(0x00, &id, 1);
    }

    ESP_LOGI(TAG, "QMI8658 OK!");

    qmi8658_register_write_byte(QMI8658_RESET, 0xb0); // 复位
    vTaskDelay(10 / portTICK_PERIOD_MS);
    qmi8658_register_write_byte(QMI8658_CTRL1, 0x40); // CTRL1 设置地址自动增加
    qmi8658_register_write_byte(QMI8658_CTRL7, 0x03); // CTRL7 允许加速度和陀螺仪
    qmi8658_register_write_byte(QMI8658_CTRL2, 0x95); // CTRL2 设置ACC 4g 250Hz
    qmi8658_register_write_byte(QMI8658_CTRL3, 0xd5); // CTRL3 设置GRY 512dps 250Hz
}

// 读取加速度和陀螺仪寄存器值
void qmi8658_Read_AccAndGry(t_sQMI8658 *p)
{
    uint8_t status, data_ready = 0;
    int16_t buf[6];

    qmi8658_register_read(QMI8658_STATUS0, &status, 1); // 读状态寄存器
    if (status & 0x03)                                  // 判断加速度和陀螺仪数据是否可读
        data_ready = 1;
    if (data_ready == 1)
    { // 如果数据可读
        data_ready = 0;
        qmi8658_register_read(QMI8658_AX_L, (uint8_t *)buf, 12); // 读加速度和陀螺仪值
        p->acc_x = buf[0];
        p->acc_y = buf[1];
        p->acc_z = buf[2];
        p->gyr_x = buf[3];
        p->gyr_y = buf[4];
        p->gyr_z = buf[5];
    }
}
// 获取XYZ轴的倾角值
void qmi8658_fetch_angleFromAcc(t_sQMI8658 *p)
{
    float temp;
    qmi8658_Read_AccAndGry(p); // 读取加速度和陀螺仪的寄存器值
    // 根据寄存器值 计算倾角值 并把弧度转换成角度
    temp = (float)p->acc_x / sqrt(((float)p->acc_y * (float)p->acc_y + (float)p->acc_z * (float)p->acc_z));
    p->AngleX = atan(temp) * 57.29578f; // 180/π=57.29578
    temp = (float)p->acc_y / sqrt(((float)p->acc_x * (float)p->acc_x + (float)p->acc_z * (float)p->acc_z));
    p->AngleY = atan(temp) * 57.29578f; // 180/π=57.29578
    temp = sqrt(((float)p->acc_x * (float)p->acc_x + (float)p->acc_y * (float)p->acc_y)) / (float)p->acc_z;
    p->AngleZ = atan(temp) * 57.29578f; // 180/π=57.29578
}