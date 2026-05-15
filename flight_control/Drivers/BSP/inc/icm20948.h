#ifndef __ICM20948_H
#define __ICM20948_H

#include "stm32f4xx.h"
#include "board.h"
#include "spi_sensor.h"

#define ICM20948_REG_WHO_AM_I            0x00
#define ICM20948_REG_USER_CTRL           0x03
#define ICM20948_REG_LP_CONFIG           0x02
#define ICM20948_REG_PWR_MGMT_1          0x06
#define ICM20948_REG_PWR_MGMT_2          0x07
#define ICM20948_REG_ACCEL_OUT_H         0x2D
#define ICM20948_REG_GYRO_OUT_H          0x33
#define ICM20948_REG_TEMP_OUT_H          0x39
#define ICM20948_REG_EXT_SENS_DATA_00    0x3B
#define ICM20948_REG_EXT_SENS_DATA_08    0x43
#define ICM20948_REG_BANK_SEL            0x7F

#define ICM20948_REG_GYRO_CONFIG_1       0x01
#define ICM20948_REG_ACCEL_CONFIG        0x14
#define ICM20948_REG_ACCEL_CONFIG_2      0x15

/* I2C Master 寄存器 (Bank 0) */
#define ICM20948_REG_I2C_MST_CTRL        0x24
#define ICM20948_REG_I2C_SLV0_ADDR       0x25
#define ICM20948_REG_I2C_SLV0_REG        0x26
#define ICM20948_REG_I2C_SLV0_CTRL       0x27
#define ICM20948_REG_I2C_SLV0_DO         0x63
#define ICM20948_REG_I2C_SLV1_ADDR       0x28
#define ICM20948_REG_I2C_SLV1_REG        0x29
#define ICM20948_REG_I2C_SLV1_CTRL       0x2A
#define ICM20948_REG_I2C_SLV1_DO         0x64
#define ICM20948_REG_I2C_SLV2_ADDR       0x2B
#define ICM20948_REG_I2C_SLV2_REG        0x2C
#define ICM20948_REG_I2C_SLV2_CTRL       0x2D
#define ICM20948_REG_I2C_SLV2_DO         0x65

/* Bank 3 寄存器 */
#define ICM20948_REG_I2C_MST_ODR_CONFIG  0x01    /* Bank 3 */

/* ================= AK09916C 磁力计 ================= */
#define AK09916_I2C_ADDR        0x0C
#define AK09916_REG_WIA2        0x01
#define AK09916_REG_ST1         0x10
#define AK09916_REG_HXL         0x11
#define AK09916_REG_CNTL2       0x31
#define AK09916_REG_CNTL3       0x32
#define AK09916_MODE_CONT       0x06
#define AK09916_MODE_RESET      0x01

/* ================= 控制位 ================= */
#define BIT_I2C_MST_EN          (1 << 5)
#define BIT_I2C_IF_DIS          (1 << 4)
#define BIT_I2C_MST_RST         (1 << 3)
#define BIT_I2C_MST_P_NSR       (1 << 5)
#define BIT_RESET               (1 << 7)
#define BIT_CLK_PLL             0x01


#define IMU_DMA_LEN 14
// extern uint8_t imu_tx_buf[IMU_DMA_LEN];

// extern uint8_t imu_rx_buf[IMU_DMA_LEN];
// extern uint8_t mag_rx_buf[8];

typedef struct {
    float accel_bias[3];
} accel_calib_t;

void ICM20948_Init(void);
uint8_t ICM20948_Read_WhoAmI(void);
uint8_t ICM20948_Read_MagWhoAmI(void);
void ICM20948_Read(uint8_t *imu_rx_buf, uint8_t *mag_rx_buf);

# endif /* __ICM20948_H */