#ifndef __CALIBRATE_ACCEL_H
#define __CALIBRATE_ACCEL_H

#include "stm32f4xx.h"

#define CALIB_ACCEL_FACES         6
#define CALIB_ACCEL_FACE_SAMPLES  300


// 加速度计校准参数（6面校准）
typedef struct {
    float bias[3];      // 零偏 (m/s²)
    float scale[3];     // 比例因子 (无量纲，理想为1)
    uint8_t is_valid;   // 校准是否有效
} Calib_Accel_t;

// 校准状态
typedef enum {
    CALIB_ACCEL_IDLE = 0,
    CALIB_ACCEL_COLLECTING,
    CALIB_ACCEL_DONE,
    CALIB_ACCEL_FAILED
} Calib_Accel_State_t;

// 校准句柄
typedef struct {
    Calib_Accel_t calib;
    Calib_Accel_State_t state;

    float face_min[CALIB_ACCEL_FACES][3];  // [face][axis]
    float face_max[CALIB_ACCEL_FACES][3];  // [face][axis]
    float face_done[CALIB_ACCEL_FACES];
    uint32_t face_samples_count[CALIB_ACCEL_FACES];
    uint8_t current_face;  // 0-5
    
    float samples[CALIB_ACCEL_FACE_SAMPLES * CALIB_ACCEL_FACES][3];
    uint32_t sample_count;
} Calib_Accel_Handle_t;

// 接口函数
void Calib_Accel_Init(Calib_Accel_Handle_t *handle);
void Calib_Accel_Start(Calib_Accel_Handle_t *handle);
void Calib_Accel_AddSample(Calib_Accel_Handle_t *handle, const float accel[3], uint8_t face);
uint8_t Calib_Accel_IsFaceDone(Calib_Accel_Handle_t *handle, uint8_t face);
void Calib_Accel_Compute(Calib_Accel_Handle_t *handle);
void Calib_Accel_Apply(const Calib_Accel_t *calib, const float raw[3], float calibrated[3]);
void Calib_Accel_Load(Calib_Accel_Handle_t *handle, const Calib_Accel_t *calib);

#endif /* __CALIBRATE_ACCEL_H */