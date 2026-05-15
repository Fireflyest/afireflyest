#ifndef __CALIBRATE_GYRO_H
#define __CALIBRATE_GYRO_H

#include "stm32f4xx.h"




typedef struct {

} Calib_Gyro_t;

// 校准状态
typedef enum {
    CALIB_GYRO_IDLE = 0,
    CALIB_GYRO_COLLECTING,
    CALIB_GYRO_DONE,
    CALIB_GYRO_FAILED
} Calib_Gyro_State_t;

// 校准句柄
typedef struct {
    Calib_Gyro_t calib;
    Calib_Gyro_State_t state;


    uint32_t sample_count;
} Calib_Gyro_Handle_t;

// 接口函数
void Calib_Gyro_Init(Calib_Gyro_Handle_t *handle);
void Calib_Gyro_Start(Calib_Gyro_Handle_t *handle);
void Calib_Gyro_AddSample(Calib_Gyro_Handle_t *handle, const float gyro[3]);
void Calib_Gyro_Compute(Calib_Gyro_Handle_t *handle);
void Calib_Gyro_Apply(const Calib_Gyro_t *calib, const float raw[3], float calibrated[3]);
void Calib_Gyro_Load(Calib_Gyro_Handle_t *handle, const Calib_Gyro_t *calib);

#endif /* __CALIBRATE_GYRO_H */