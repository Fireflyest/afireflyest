#ifndef __MPU_H
#define __MPU_H

#include "stm32f4xx.h"

// #define COMMUNICATION_TYPE_I2C
#define COMMUNICATION_TYPE_SPI

// #define MPU_6500
// #define MPU_9250
#define ICM_20948
#define BMP280

#if defined(MPU_6500) || defined(MPU_9250)
#define	MPU_SMPLRT_DIV		0x19
#define	MPU_CONFIG			0x1A
#define	MPU_GYRO_CONFIG		0x1B
#define	MPU_ACCEL_CONFIG	0x1C

#define	MPU_ACCEL_XOUT_H	0x3B
#define	MPU_ACCEL_XOUT_L	0x3C
#define	MPU_ACCEL_YOUT_H	0x3D
#define	MPU_ACCEL_YOUT_L	0x3E
#define	MPU_ACCEL_ZOUT_H	0x3F
#define	MPU_ACCEL_ZOUT_L	0x40
#define	MPU_TEMP_OUT_H		0x41
#define	MPU_TEMP_OUT_L		0x42
#define	MPU_GYRO_XOUT_H		0x43
#define	MPU_GYRO_XOUT_L		0x44
#define	MPU_GYRO_YOUT_H		0x45
#define	MPU_GYRO_YOUT_L		0x46
#define	MPU_GYRO_ZOUT_H		0x47
#define	MPU_GYRO_ZOUT_L		0x48

#define	MPU_PWR_MGMT_1		0x6B
#define	MPU_PWR_MGMT_2		0x6C

#define MPU_ADDRESS         0xD0
#endif // #if defined(MPU_6500) || defined(MPU_9250)

#ifdef MPU_6500
// #define	MPU_WHO_AM_I		0x75
#endif // #ifdef MPU_6500

#ifdef MPU_9250
// #define MAG_WHO_AM_I        0x00
#define MAG_ADDRESS         0x0C
#define MAG_CNTL1           0x0A
#define MAG_ST1             0x02
#define MAG_XOUT_L          0x03
#endif // #ifdef MPU_9250

#ifdef ICM_20948
// #define ICM_WHO_AM_I        0xEA

#define REG_BANK_SEL        0x7F

#define ICM_PWR_MGMT_1      0x06
#define ICM_PWR_MGMT_2      0x07

#define	ICM_ACCEL_XOUT_H	0x2D
#define	ICM_ACCEL_XOUT_L	0x2E
#define	ICM_ACCEL_YOUT_H	0x2F
#define	ICM_ACCEL_YOUT_L	0x30
#define	ICM_ACCEL_ZOUT_H	0x31
#define	ICM_ACCEL_ZOUT_L	0x32

#define	ICM_GYRO_XOUT_H		0x33
#define	ICM_GYRO_XOUT_L		0x34
#define	ICM_GYRO_YOUT_H		0x35
#define	ICM_GYRO_YOUT_L		0x36
#define	ICM_GYRO_ZOUT_H		0x37
#define	ICM_GYRO_ZOUT_L		0x38

#define	ICM_TEMP_OUT_H		0x39
#define	ICM_TEMP_OUT_L		0x3A

#define ICM_GYRO_CONFIG_1   0x01
#define ICM_ACCEL_CONFIG    0x14

#define	MAG_ADDRESS_ICM		0x0C 

#endif // #ifdef ICM_20948

#ifdef BMP280
#define BMP_CTRL_MEAS      0xF4
#define BMP_CONFIG         0xF5
#endif // #ifdef BMP280

#define SEA_LEVEL_PRESSURE_HPA    1013.25f
#define BAROMETERIC_PRESSURE_OFFSET  -2.4f  // in hPa

extern uint8_t mpuDataBuffer[14];
extern uint8_t magDataBuffer[7];
extern uint8_t bmp_rx_buf[6];

typedef struct {
    float accel_bias[3];
    float accel_scale[3];
    float gyro_bias[3];
    float mag_hard[3];
    float mag_soft[3][3];
} IMUCalibrationData_t;
extern IMUCalibrationData_t imuCalibData;

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp_calib_t;
extern bmp_calib_t bmp_calib;


void Init_IMU_Hardware(void);

void Write_MPU_Register(uint8_t reg, uint8_t data);
void BMP280_Register_Write(uint8_t reg, uint8_t data);
uint8_t Read_MPU_Register(uint8_t reg);
uint8_t BMP280_Register_Read(uint8_t reg);
void Read_IMU_All();
void Read_BMP_All();

#endif /* __MPU_H */