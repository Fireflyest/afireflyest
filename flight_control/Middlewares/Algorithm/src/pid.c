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
    pid->d_max = 10000.0f; /* 兜底值; 应按实例量纲在调用处覆盖 (见 Control_Init) */
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->aw_gain = aw_gain;
    pid->start = 0;
}

/**
 * @brief PID 核心: P + I(限幅) + D(测量微分/滤波/限幅) + 输出限幅 + 反算 anti-windup
 *
 * @param dmeasured_dt  d(measured)/dt, 由调用方提供:
 *                      PID_Update 内部差分, PID_UpdateWithDeriv 用外部状态量
 *
 * D 项取 -dmeasured_dt (测量微分): 对测量构成负反馈,
 * 目标值突变时不产生设定值突跳 (no setpoint kick)。
 */
static float pid_apply(PID_t* pid, float target, float measured,
                       float dmeasured_dt, float dt) {
    /* dt 判断写成 !(dt > 0): 顺带拒绝 NaN (dt <= 0 判不出 NaN) */
    if (!isfinite(target) || !isfinite(measured) || !(dt > 0.0f)) {
        return 0.0f;
    }
    /* D 输入异常只丢 D 项, 不拖垮 P/I */
    if (!isfinite(dmeasured_dt)) {
        dmeasured_dt = 0.0f;
    }

    float err = target - measured;

    /* 首次调用初始化，避免 D 项/初始误差突变 */
    if (!pid->start) {
        pid->last_meas = measured;
        pid->last_error = err;
        pid->d_state = 0.0f;
        pid->start = 1;
    }

    /* P */
    float P = pid->kp * err;

    /* I (积分并限幅) */
    pid->integrator += pid->ki * err * dt;
    pid->integrator = fmaxf(fminf(pid->integrator, pid->integrator_max), pid->integrator_min);

    /* D: 一阶低通 + 实例限幅 (kd == 0 时整段跳过 — 全机 7 个环 × 200 Hz) */
    float D = 0.0f;
    if (pid->kd != 0.0f) {
        float deriv = -dmeasured_dt;

        if (pid->d_max > 0.0f) {
            if (deriv >  pid->d_max) deriv =  pid->d_max;
            if (deriv < -pid->d_max) deriv = -pid->d_max;
        }

        if (pid->d_tau > 0.0f) {
            float alpha = dt / (pid->d_tau + dt);
            pid->d_state = alpha * deriv + (1.0f - alpha) * pid->d_state;
            deriv = pid->d_state;
        }

        D = pid->kd * deriv;
    }

    /* 限幅输出 */
    float unclamped = P + pid->integrator + D;
    float out = fmaxf(fminf(unclamped, pid->out_max), pid->out_min);

    /* anti-windup: back-calculation — 输出被截断的偏差回灌积分器 */
    if (pid->aw_gain != 0.0f) {
        pid->integrator += pid->aw_gain * (out - unclamped) * dt;
        pid->integrator = fmaxf(fminf(pid->integrator, pid->integrator_max), pid->integrator_min);
    }

    pid->last_error = err;
    pid->last_meas = measured;
    return out;
}

float PID_Update(PID_t* pid, float target, float measured, float dt) {
    /*
     * 数值差分 (仅 kd != 0 才需要, 省一次除法):
     *   首帧 start == 0 → dmeas = 0, 由 pid_apply 完成初始化
     */
    float dmeas = 0.0f;
    if (pid->kd != 0.0f && pid->start &&
        isfinite(target) && isfinite(measured) && dt > 0.0f) {
        dmeas = (measured - pid->last_meas) / dt;
        if (!isfinite(dmeas)) dmeas = 0.0f;
    }
    return pid_apply(pid, target, measured, dmeas, dt);
}

float PID_UpdateWithDeriv(PID_t* pid, float target, float measured,
                          float dmeasured_dt, float dt) {
    return pid_apply(pid, target, measured, dmeasured_dt, dt);
}

void PID_Reset(PID_t* pid) {
    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->last_meas = 0.0f;
    pid->d_state = 0.0f;
    pid->start = 0;
}
