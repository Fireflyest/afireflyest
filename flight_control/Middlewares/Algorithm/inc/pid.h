#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

typedef struct {
    float kp, ki, kd;
    float integrator;
    float integrator_min, integrator_max; // 积分限幅
    float last_error;
    float last_meas;      // 用于测量微分，避免微分冲击
    float d_state;        // D 滤波状态
    float d_tau;          // D 项一阶滤波时间常数 (s), 0 表示不滤波
    float out_min, out_max; // 输出限幅
    float aw_gain;        // anti-windup 回馈增益（通常 0.5~2.0）
    uint8_t start;
} PID_t;

void PID_Init(PID_t* pid, float kp, float ki, float kd,
              float integrator_min, float integrator_max,
              float d_tau,
              float out_min, float out_max,
              float aw_gain);

float PID_Update(PID_t* pid, float target, float measured, float dt);
void PID_Reset(PID_t* pid);

#endif /* __PID_H */