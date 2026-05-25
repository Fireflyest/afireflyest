#ifndef __ACCEL_GYRO_H
#define __ACCEL_GYRO_H

#include "stm32f4xx.h"

typedef struct {
    int16_t x, y, z;
} bmi_raw_data_t;

/**
 * @brief  初始化 BMI260（Bosch 驱动 + ACC/GYR 配置）
 * @param  spi_id  SPI_Sensor_Register() 返回的 ID
 * @return 0=成功, 负值=Bosch 错误码
 */
int BMI_Init(uint8_t spi_id);

/**
 * @brief  读取芯片 ID（SPI dummy byte 已处理）
 * @return 0=正确(0x27), -1=通信失败, -2=ID 不匹配
 */
int BMI_WhoAmI(uint8_t* id);

/**
 * @brief  同时读取加速度计和陀螺仪
 * @return 0=成功
 */
int BMI_ReadAccelGyro(bmi_raw_data_t* acc, bmi_raw_data_t* gyr);

int BMI_ReadAccel(bmi_raw_data_t* data);
int BMI_ReadGyro(bmi_raw_data_t* data);

/**
 * @brief  物理量转换（与官方 lsb_to_mps2 / lsb_to_dps 一致）
 *         bit_width = bmi.resolution，初始化后自动获取
 */
void BMI_AccelToMps2(const bmi_raw_data_t* raw, float g_range, uint8_t bit_width, float out_mps2[3]);
void BMI_GyroToRads(const bmi_raw_data_t* raw, float dps, uint8_t bit_width, float out_rads[3]);

/**
 * @brief  打印 Bosch 错误码
 */
void BMI_PrintError(int8_t rslt);

/**
 * @brief  获取内部 bmi2_dev 指针（高级用途）
 */
struct bmi2_dev* BMI_GetDev(void);

#endif /* __ACCEL_GYRO_H */
