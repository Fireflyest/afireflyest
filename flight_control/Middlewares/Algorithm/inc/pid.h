#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

/**
 * @brief 输出不限幅哨兵值: 传给出/入限幅即可关闭输出钳位
 * @note  输出不限幅时 back-calculation anti-windup 不再触发 (无饱和可回灌),
 *        积分器只能靠 integrator_min/max 把守 —— 这两个不要跟着放开
 */
#define PID_OUT_UNLIMITED 3.4e38f

typedef struct {
    float kp, ki, kd;
    float integrator;
    float integrator_min, integrator_max; // 积分限幅
    float last_error;
    float last_meas;      // 用于测量微分，避免微分冲击
    float d_state;        // D 滤波状态
    float d_tau;          // D 项一阶滤波时间常数 (s), 0 表示不滤波
    float d_max;          // 测量微分限幅 (量纲: measured/s), <=0 表示不限幅
    float out_min, out_max; // 输出限幅
    float aw_gain;        // anti-windup 回馈增益（通常 0.5~2.0）
    uint8_t start;
} PID_t;

void PID_Init(PID_t* pid, float kp, float ki, float kd,
              float integrator_min, float integrator_max,
              float d_tau,
              float out_min, float out_max,
              float aw_gain);

/**
 * @brief 常规 PID 更新, 微分由内部对 measured 数值差分得到
 * @note  kd == 0 时跳过差分与滤波计算
 */
float PID_Update(PID_t* pid, float target, float measured, float dt);

/**
 * @brief PID 更新, 由调用方提供测量导数 (避免对噪声测量做数值微分)
 * @param dmeasured_dt  d(measured)/dt, 量纲 = measured 单位/s
 *                      例: 高度环传 EKF 垂直速度 (m/s, 向上为正)
 * @note  非法 (NaN) 导数按 0 处理, 只丢 D 项, 不影响 P/I
 */
float PID_UpdateWithDeriv(PID_t* pid, float target, float measured,
                          float dmeasured_dt, float dt);
void PID_Reset(PID_t* pid);

#endif /* __PID_H */