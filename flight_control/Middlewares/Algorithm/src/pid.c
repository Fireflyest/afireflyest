#include "pid.h"
#include <math.h>

void PID_Init(PID_t* pid, float kp, float ki, float kd,
              float integrator_min, float integrator_max,
              float d_tau,
              float out_min, float out_max,
              float aw_gain) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->integrator_min = integrator_min;
    pid->integrator_max = integrator_max;
    pid->last_error = 0.0f;
    pid->last_meas = 0.0f;
    pid->d_state = 0.0f;
    pid->d_tau = d_tau;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->aw_gain = aw_gain;
    pid->start = 0;
}


float PID_Update(PID_t* pid, float target, float measured, float dt) {
    if (!isfinite(target) || !isfinite(measured) || dt <= 0.0f) {
        return 0.0f;
    }
    float err = target - measured;

    // 首次调用初始化，避免 D 项/初始误差突变
    if (!pid->start) {
        pid->last_meas = measured;
        pid->last_error = err;
        pid->d_state = 0.0f;
        pid->start = 1;
    }

    // P
    float P = pid->kp * err;

    // I (积分并限幅)
    pid->integrator += pid->ki * err * dt;
    pid->integrator = fmaxf(fminf(pid->integrator, pid->integrator_max), pid->integrator_min);

    // D: 用测量微分 + 一阶低通 + 限幅保护
    float deriv = -(measured - pid->last_meas) / dt;
    if (!isfinite(deriv)) deriv = 0.0f;

    // 限幅 deriv 防止异常或采样跳变影响
    const float MAX_DERIV = 10000.0f; // 根据实际量纲调整（较大值最多保护）
    if (deriv >  MAX_DERIV) deriv =  MAX_DERIV;
    if (deriv < -MAX_DERIV) deriv = -MAX_DERIV;

    if (pid->d_tau > 0.0f) {
        float alpha = dt / (pid->d_tau + dt);
        pid->d_state = alpha * deriv + (1.0f - alpha) * pid->d_state;
        deriv = pid->d_state;
    }

    float D = pid->kd * deriv;

    // 未限幅输出
    float unclamped = P + pid->integrator + D;

    // 限幅输出
    float out = fmaxf(fminf(unclamped, pid->out_max), pid->out_min);

    // anti-windup：back-calculation（把输出被截断的误差反馈到积分器）
    float diff = out - unclamped; // negative/positive if saturated
    if (pid->aw_gain != 0.0f) {
        pid->integrator += pid->aw_gain * diff * dt;
        pid->integrator = fmaxf(fminf(pid->integrator, pid->integrator_max), pid->integrator_min);
    }

    pid->last_error = err;
    pid->last_meas = measured;
    return out;
}

void PID_Reset(PID_t* pid) {
    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->last_meas = 0.0f;
    pid->d_state = 0.0f;
    pid->start = 0;
}