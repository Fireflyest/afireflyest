/**
 * ============================================================================
 *  control.c — 四旋翼级联 PID 飞行控制器
 * ============================================================================
 *
 *  一、主要内容
 *  ─────────────────────────────────────────────────────────────────────────
 *    - 全局目标四元数 targetQuat（机体坐标系）
 *    - 四元数误差计算替代欧拉角直接相减
 *    - 陀螺仪偏置补偿（通过 attitude 接口）
 *    - 所有四元数运算使用 spatial_math.h
 *
 *  二、四元数误差计算
 *  ─────────────────────────────────────────────────────────────────────────
 *    目标四元数 targetQuat（由目标欧拉角构建）
 *    当前四元数 curQuat（从 attitude 接口读取）
 *
 *    误差四元数（机体坐标系中的相对旋转）：
 *      q_err = conj(q_target) × q_cur
 *
 *    从 q_err 提取欧拉角 → 送入角度环 PID
 *
 *  三、陀螺仪偏置补偿
 *  ─────────────────────────────────────────────────────────────────────────
 *    EKF 估计的陀螺仪偏置 [bx, by, bz]
 *    补偿后：gyro_corrected = gyro_raw - bias
 *
 * ============================================================================
 */

#include "control.h"
#include <math.h>
#include "attitude.h"
#include "pwm.h"
#include "spatial_math.h"

/* ══════════════════════════════════════════════════════════════
 *  配置常量
 * ══════════════════════════════════════════════════════════════ */

#define THROTTLE_DEADBAND 2.0f
#define GIMBAL_LOCK_THRESH 0.95f /* sin(pitch) > 此值时禁用 yaw */
#define M_PI_F 3.14159265f

/* ══════════════════════════════════════════════════════════════
 *  PID & 公开变量
 * ══════════════════════════════════════════════════════════════ */

PID_t pidRoll, pidPitch, pidYaw, pidHeight;
PID_t pidRateRoll, pidRatePitch, pidRateYaw;
float baseThrottle = 30.0f;

/* 临界区保护的共享变量 */
__IO float rateSetRoll, rateSetPitch, rateSetYaw;
__IO float thrustOutput;

/* ══════════════════════════════════════════════════════════════
 *  内部状态
 * ══════════════════════════════════════════════════════════════ */

static ControlMode_t curMode = CONTROL_MODE_DIRECT;
static FlightPhase_t curPhase = FLIGHT_PHASE_GROUNDED;
static volatile uint8_t isArmed = 0;

static float baseHeight = 0.0f;
static float targetRoll = 0.0f;
static float targetPitch = 0.0f;
static float targetYaw = 0.0f;
static float targetHeight = 0.0f;
static float moveForward = 0.0f;
static float moveRight = 0.0f;
static float rampedHeight = 0.0f;
static uint8_t ramp_init = 0;

/* 目标四元数（机体坐标系，Hamilton [w,x,y,z]）*/
static sm_quat_t targetQuat = {1.0f, 0.0f, 0.0f, 0.0f};

/* ══════════════════════════════════════════════════════════════
 *  内部工具
 * ══════════════════════════════════════════════════════════════ */

static float NormalizeAngle(float a) {
    while (a > 180.0f)
        a -= 360.0f;
    while (a <= -180.0f)
        a += 360.0f;
    return a;
}

static void ResetAllPIDs(void) {
    PID_Reset(&pidRoll);
    PID_Reset(&pidPitch);
    PID_Reset(&pidYaw);
    PID_Reset(&pidHeight);
    PID_Reset(&pidRateRoll);
    PID_Reset(&pidRatePitch);
    PID_Reset(&pidRateYaw);
}

static void ResetAllTargets(void) {
    targetRoll = 0.0f;
    targetPitch = 0.0f;
    targetYaw = 0.0f;
    targetHeight = baseHeight;
    moveForward = 0.0f;
    moveRight = 0.0f;
    targetQuat[0] = 1.0f;
    targetQuat[1] = 0.0f;
    targetQuat[2] = 0.0f;
    targetQuat[3] = 0.0f;
}

static void StopMotors(void) {
    __disable_irq();
    thrustOutput = 0.0f;
    rateSetRoll = 0.0f;
    rateSetPitch = 0.0f;
    rateSetYaw = 0.0f;
    __enable_irq();

    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
}

/* ══════════════════════════════════════════════════════════════
 *  外环（姿态 + 高度）— 由主循环以 200 Hz 调用
 * ══════════════════════════════════════════════════════════════ */

void ControlAttitude_Loop(void) {
    if (!isArmed) {
        StopMotors();
        return;
    }

    const float dt = 1.0f / (float)ATTITUDE_LOOP_HZ;

    /* ── 读取传感器四元数 ────────────────────────── */
    sm_quat_t curQuat;
    Attitude_GetQuat(curQuat);

    if (!isfinite(curQuat[0]) || !isfinite(curQuat[1]) ||
        !isfinite(curQuat[2]) || !isfinite(curQuat[3])) {
        return;
    }

    /* ── 解析当前欧拉角（机体坐标系） ────────────── */
    float curRoll, curPitch, curYaw;
    Spatial_QuatGetEuler(&curYaw, &curPitch, &curRoll, curQuat);
    curRoll *= (180.0f / M_PI_F);
    curPitch *= (180.0f / M_PI_F);
    curYaw *= (180.0f / M_PI_F);

    float curHeight;
    Attitude_GetAltitude(&curHeight);

    /* ── 降落状态机 ───────────────────────────────── */
    if (curPhase == FLIGHT_PHASE_LANDING) {
        if (curHeight < baseHeight + 0.05f) {
            curPhase = FLIGHT_PHASE_GROUNDED;
            Control_Disarm();
            return;
        }
    }

    /* ── 起飞状态机 ───────────────────────────────── */
    if (curPhase == FLIGHT_PHASE_TAKING_OFF) {
        if (fabsf(curHeight - targetHeight) < 0.10f) {
            curPhase = FLIGHT_PHASE_IN_FLIGHT;
        }
    }

    /* ── 高度环 ──────────────────────────────────── */
    if (curMode >= CONTROL_MODE_ALTITUDE) {
        if (!ramp_init) {
            rampedHeight = baseHeight;
            ramp_init = 1;
        }

        float heightError = targetHeight - rampedHeight;
        float rampSpeed = 0.3f;  // m/s，慢慢升
        if (heightError > rampSpeed * dt) {
            rampedHeight += rampSpeed * dt;
        } else if (heightError < -rampSpeed * dt) {
            rampedHeight -= rampSpeed * dt;
        } else {
            rampedHeight = targetHeight;
        }

        thrustOutput = baseThrottle + PID_Update(&pidHeight, rampedHeight, curHeight, dt);
        thrustOutput = fmaxf(0.0f, fminf(thrustOutput, 100.0f));
    }

    /* ── DIRECT 模式：不输出角度环 ────────────────── */
    if (curMode == CONTROL_MODE_DIRECT) {
        __disable_irq();
        rateSetRoll = 0.0f;
        rateSetPitch = 0.0f;
        rateSetYaw = 0.0f;
        __enable_irq();
        return;
    }

    /* ── 姿态指令（加入平移叠加） ────────────────── */
    float effRoll = targetRoll;
    float effPitch = targetPitch;

    if (curMode >= CONTROL_MODE_ALTITUDE) {
        effPitch -= moveForward * 25.0f;
        effRoll += moveRight * 25.0f;
    }

    /* ── 构建目标四元数（机体坐标系） ────────────── */
    float yawErrDeg = NormalizeAngle(targetYaw - curYaw);
    float effYaw = NormalizeAngle(curYaw + yawErrDeg);

    Spatial_QuatFromEuler(targetQuat,
                          effYaw * (M_PI_F / 180.0f),
                          effPitch * (M_PI_F / 180.0f),
                          effRoll * (M_PI_F / 180.0f));
    Spatial_QuatNormalize(targetQuat);

    /* ── 四元数误差计算 ──────────────────────────── */
    sm_quat_t qTargetInv = {targetQuat[0], targetQuat[1], targetQuat[2], targetQuat[3]};
    Spatial_QuatConjugate(qTargetInv);

    sm_quat_t qErr;
    Spatial_QuatMultiply(qErr, qTargetInv, curQuat);
    Spatial_QuatNormalize(qErr);

    /* 最短路径保护：qErr.w < 0 表示走了长路径 */
    if (qErr[0] < 0.0f) {
        qErr[0] = -qErr[0];
        qErr[1] = -qErr[1];
        qErr[2] = -qErr[2];
        qErr[3] = -qErr[3];
    }

    float errRoll, errPitch, errYaw;
    Spatial_QuatGetEuler(&errYaw, &errPitch, &errRoll, qErr);
    errRoll *= (180.0f / M_PI_F);
    errPitch *= (180.0f / M_PI_F);
    errYaw *= (180.0f / M_PI_F);

    /* Gimbal Lock 保护 */
    float sinPitch = 2.0f * (curQuat[0] * curQuat[2] + curQuat[1] * curQuat[3]);
    if (fabsf(sinPitch) > GIMBAL_LOCK_THRESH) {
        errYaw = 0.0f;
    }

    /* ── 角度环 PID ──────────────────────────────── */
    float newRateSetRoll = PID_Update(&pidRoll, 0.0f, errRoll, dt);
    float newRateSetPitch = PID_Update(&pidPitch, 0.0f, errPitch, dt);
    float newRateSetYaw = PID_Update(&pidYaw, 0.0f, errYaw, dt);

    /* 原子写入共享变量 */
    __disable_irq();
    rateSetRoll = newRateSetRoll;
    rateSetPitch = newRateSetPitch;
    rateSetYaw = newRateSetYaw;
    __enable_irq();
}

/* ══════════════════════════════════════════════════════════════
 *  内环（速率）— 由 TIM4 中断调用
 * ══════════════════════════════════════════════════════════════ */

void ControlMotor_Loop(void) {
    if (!isArmed) {
        TIM3->CCR1 = 0;
        TIM3->CCR2 = 0;
        TIM3->CCR3 = 0;
        TIM3->CCR4 = 0;
        return;
    }

    const float dt = 1.0f / (float)RATE_LOOP_HZ;

    /* 原子读取共享变量 */
    float localRateSetRoll, localRateSetPitch, localRateSetYaw;
    __disable_irq();
    localRateSetRoll = rateSetRoll;
    localRateSetPitch = rateSetPitch;
    localRateSetYaw = rateSetYaw;
    __enable_irq();

    /* ── 读取陀螺仪 ──────────────────────────────── */
    sm_vec3_t gyro;
    Attitude_GetGyro(gyro);

    float gx = gyro[0] * (180.0f / M_PI_F);
    float gy = gyro[1] * (180.0f / M_PI_F);
    float gz = gyro[2] * (180.0f / M_PI_F);

    /* ── 速率环 PID ──────────────────────────────── */
    // float PID_Update(PID_t *pid, float target, float measured, float dt)
    float rollCtrl = PID_Update(&pidRateRoll, localRateSetRoll, gx, dt);
    float pitchCtrl = PID_Update(&pidRatePitch, localRateSetPitch, gy, dt);
    float yawCtrl = PID_Update(&pidRateYaw, localRateSetYaw, gz, dt);

    float throttle = thrustOutput;

    /* 电机混控 (FRD X型四旋翼)
     * M2: 前左(FL)   M1: 后左(RL)
     * M4: 前右(FR)   M3: 后右(RR)
     *
     * +Pitch → 后方电机加速 → M2+, M4+ / M1-, M3-
     * +Roll  → 右侧电机加速 → M3+, M4+ / M1-, M2-
     * +Yaw   → CCW电机加速  → M2+, M3+ / M1-, M4-
     */
    float m[4];
    m[0] = throttle + pitchCtrl + rollCtrl + yawCtrl;  // M1: 后左 (RL, CW)
    m[1] = throttle - pitchCtrl + rollCtrl - yawCtrl;  // M2: 前左 (FL, CCW)
    m[2] = throttle + pitchCtrl - rollCtrl - yawCtrl;  // M3: 后右 (RR, CCW)
    m[3] = throttle - pitchCtrl - rollCtrl + yawCtrl;  // M4: 前右 (FR, CW)

    /* 只绕 X 轴：左侧和右侧反向 */
    // float m[4];
    // m[0] = throttle + rollCtrl;  // M0: 左侧
    // m[1] = throttle + rollCtrl;  // M1: 左侧
    // m[2] = throttle - rollCtrl;  // M2: 右侧
    // m[3] = throttle - rollCtrl;  // M3: 右侧

    TIM3->CCR1 = PWM_Map_Percent(m[0]);
    TIM3->CCR2 = PWM_Map_Percent(m[1]);
    TIM3->CCR3 = PWM_Map_Percent(m[2]);
    TIM3->CCR4 = PWM_Map_Percent(m[3]);
}

/* ══════════════════════════════════════════════════════════════
 *  初始化
 * ══════════════════════════════════════════════════════════════ */

void Control_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TB;
    uint32_t timer_clk = SystemCoreClock;
    uint16_t presc = (uint16_t)(timer_clk / 1000000UL) - 1;
    uint16_t period = (uint16_t)(1000000UL / RATE_LOOP_HZ) - 1;

    TIM_TimeBaseStructInit(&TB);
    TB.TIM_Prescaler = presc;
    TB.TIM_CounterMode = TIM_CounterMode_Up;
    TB.TIM_Period = period;
    TB.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TB);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);

    /* 角度环 */
    // float PID_Update(PID_t *pid, float target, float measured, float dt)
    PID_Init(&pidHeight, 10.0f, 1.0f, 6.0f, -15.0f, 15.0f, 0.02f, -40.0f, 40.0f, 1.0f);
    PID_Init(&pidRoll, 4.0f, 0.01f, 0.0f, -20.0f, 20.0f, 0.02f, -100.0f, 100.0f, 1.0f);
    PID_Init(&pidPitch, 4.0f, 0.01f, 0.0f, -20.0f, 20.0f, 0.02f, -100.0f, 100.0f, 1.0f);
    PID_Init(&pidYaw, 2.0f, 0.01f, 0.0f, -20.0f, 20.0f, 0.02f, -100.0f, 100.0f, 1.0f);

    /* 速率环 */
    PID_Init(&pidRateRoll, 0.4f, 0.3f, 0.008f, -20.0f, 20.0f, 0.01f, -100.0f, 100.0f, 1.0f);
    PID_Init(&pidRatePitch, 0.4f, 0.3f, 0.008f, -20.0f, 20.0f, 0.01f, -100.0f, 100.0f, 1.0f);
    PID_Init(&pidRateYaw, 0.1f, 0.05f, 0.0f, -20.0f, 20.0f, 0.01f, -100.0f, 100.0f, 1.0f);
}

/* ══════════════════════════════════════════════════════════════
 *  模式切换
 * ══════════════════════════════════════════════════════════════ */

int8_t Control_SetMode(ControlMode_t mode) {
    if (mode > CONTROL_MODE_POSITION)
        return -1;

    curMode = mode;
    moveForward = 0.0f;
    moveRight = 0.0f;
    ResetAllPIDs();

    return mode;
}

ControlMode_t Control_GetMode(void) {
    return curMode;
}

FlightPhase_t Control_GetFlightPhase(void) {
    return curPhase;
}

float Control_GetBaseHeight(void) {
    return baseHeight;
}

void Control_GetTargetQuat(sm_quat_t out) {
    out[0] = targetQuat[0];
    out[1] = targetQuat[1];
    out[2] = targetQuat[2];
    out[3] = targetQuat[3];
}

/* ══════════════════════════════════════════════════════════════
 *  解锁 / 锁定
 * ══════════════════════════════════════════════════════════════ */

int8_t Control_Arm(void) {
    if (curMode != CONTROL_MODE_DIRECT)
        return -1;
    if (thrustOutput > 1.0f)
        return -1;

    StopMotors();
    ResetAllPIDs();
    ResetAllTargets();

    Attitude_GetAltitude(&baseHeight);

    curPhase = FLIGHT_PHASE_GROUNDED;
    isArmed = 1;

    return curPhase;
}

uint8_t Control_IsArmed(void) {
    return isArmed;
}

int8_t Control_Disarm(void) {
    isArmed = 0;
    ramp_init = 0;

    StopMotors();
    ResetAllPIDs();
    ResetAllTargets();

    curPhase = FLIGHT_PHASE_GROUNDED;

    return 0;
}

void Control_EmergencyStop(void) {
    isArmed = 0;

    StopMotors();
    ResetAllPIDs();
    ResetAllTargets();

    curPhase = FLIGHT_PHASE_GROUNDED;
}

/* ══════════════════════════════════════════════════════════════
 *  飞行操作
 * ══════════════════════════════════════════════════════════════ */

int8_t Control_Takeoff(float relative_height) {
    if (!isArmed)
        return -1;
    if (curPhase != FLIGHT_PHASE_GROUNDED)
        return -1;
    if (relative_height < 0.1f)
        return -1;

    targetHeight = baseHeight + relative_height;
    targetRoll = 0.0f;
    targetPitch = 0.0f;
    moveForward = 0.0f;
    moveRight = 0.0f;
    ramp_init = 0;

    if (curMode < CONTROL_MODE_ALTITUDE) {
        Control_SetMode(CONTROL_MODE_ALTITUDE);
    }

    curPhase = FLIGHT_PHASE_TAKING_OFF;

    return curPhase;
}

int8_t Control_Land(void) {
    if (!isArmed)
        return -1;
    if (curPhase == FLIGHT_PHASE_GROUNDED)
        return -1;
    if (curPhase == FLIGHT_PHASE_LANDING)
        return 0;

    targetHeight = baseHeight;
    targetRoll = 0.0f;
    targetPitch = 0.0f;
    moveForward = 0.0f;
    moveRight = 0.0f;

    curPhase = FLIGHT_PHASE_LANDING;

    return curPhase;
}

void Control_Hover(void) {
    if (curMode < CONTROL_MODE_ALTITUDE)
        return;

    moveForward = 0.0f;
    moveRight = 0.0f;
    targetRoll = 0.0f;
    targetPitch = 0.0f;
}

/* ══════════════════════════════════════════════════════════════
 *  指令输入
 * ══════════════════════════════════════════════════════════════ */

void Control_SetThrottle(float throttle) {
    if (!isArmed) {
        thrustOutput = 0.0f;
        return;
    }

    if (curMode > CONTROL_MODE_STABILIZED)
        return;

    if (throttle < THROTTLE_DEADBAND)
        throttle = 0.0f;
    thrustOutput = fmaxf(0.0f, fminf(throttle, 100.0f));
}

void Control_SetAttitude(float roll, float pitch, float yaw) {
    if (curMode == CONTROL_MODE_DIRECT)
        return;

    targetRoll = fmaxf(-45.0f, fminf(roll, 45.0f));
    targetPitch = fmaxf(-45.0f, fminf(pitch, 45.0f));
    targetYaw = NormalizeAngle(yaw);
}

void Control_Move(float forward, float right) {
    if (curMode < CONTROL_MODE_ALTITUDE)
        return;

    moveForward = fmaxf(-1.0f, fminf(forward, 1.0f));
    moveRight = fmaxf(-1.0f, fminf(right, 1.0f));
}

void Control_SetHeight(float height) {
    if (curMode < CONTROL_MODE_ALTITUDE)
        return;

    targetHeight = fmaxf(height, baseHeight);
}
