/**
 * @file control.c
 * @brief 四旋翼飞行控制模块 — 完整实现
 *
 * ============================================================================
 *  电机混控表 (FRD X 型四旋翼)
 * ============================================================================
 *
 *  以下 "正指令" 的物理含义:
 *      Roll  正 = 施加正滚转力矩 → 右翼下沉
 *      Pitch 正 = 施加正俯仰力矩 → 机头下沉
 *      Yaw   正 = 施加正偏航力矩 → 机头顺时针偏转 (俯视)
 *
 *  ┌───────────────────────────────────────────────────────────────┐
 *  │    指令            M1(RL,CW)  M2(FL,CCW)  M3(RR,CCW)  M4(FR,CW) │
 *  │    ─────────────  ─────────  ──────────  ──────────  ────────── │
 *  │    油门 (↑)         +1          +1          +1          +1       │
 *  │    Roll (右倾+)     +1          +1          -1          -1       │
 *  │    Pitch (上仰+)    -1          +1          -1          +1       │
 *  │    Yaw  (CW+)       -1          +1          +1          -1       │
 *  └───────────────────────────────────────────────────────────────┘
 *
 *  推导:
 *    Roll  正力矩: 右翼下沉 → 左侧电机推力更大 → M1+, M2+, M3-, M4-
 *    Pitch 正力矩: 机头上抬 → 前方电机推力更大 → M2+, M4+, M1-, M3-
 *    Yaw   正力矩: 机身 CW → CCW 电机反扭矩更大 → M2+, M3+, M1-, M4-
 *
 *  "正指令" 的物理含义:
 *      Roll  正 = 施加右倾力矩   → 右翼下沉
 *      Pitch 正 = 施加上仰力矩   → 机头上抬
 *      Yaw   正 = 施加 CW 力矩  → 机头顺时针偏转 (俯视)
 *
 * ============================================================================
 *  误差四元数计算
 * ============================================================================
 *
 *  q_err = q_target^* ⊗ q_current
 *
 *  含义: q_err 代表从目标姿态到当前姿态的旋转。
 *        当前姿态偏离目标越多，q_err 的角度越大。
 *        从 q_err 提取欧拉角即为各轴角度误差。
 *
 *  最短路径: 若 q_err.w < 0, 取反 (q 和 -q 代表同一旋转)。
 *  万向锁保护: 当 |sin(pitch)| > 0.95 时, 禁用 Yaw 修正。
 */

#include <math.h>
#include <string.h>
#include "control.h"
#include "pwm.h"

/* ========================================================================== */
/*  Section 0: 配置常量                                                        */
/* ========================================================================== */

/** @brief 沸门死区 (%) */
#define THROTTLE_DEADBAND 2.0f

/** @brief 万向锁门限 (sin(pitch) > 此值时禁用 yaw) */
#define GIMBAL_LOCK_THRESH 0.95f

/** @brief 平移倾斜限幅 (°): 满杆时叠加的最大倾角 */
#define TILT_ANGLE_LIMIT 25.0f

/** @brief 高度爬升速率限幅 (m/s) */
#define HEIGHT_RAMP_SPEED 0.3f

/** @brief 降落判据: 距基准高度 < 此值时认为已着地 (m) */
#define LANDING_HEIGHT_THRESH 0.05f

/** @brief 起飞判据: 距目标高度 < 此值时认为起飞完成 (m) */
#define TAKEOFF_HEIGHT_THRESH 0.50f

/** @brief 角度 ↔ 弧度 */
#define DEG2RAD 0.017453292f
#define RAD2DEG 57.29577951f

#ifndef M_PI_F
#define M_PI_F 3.14159265f
#endif

/* ========================================================================== */
/*  Section 1: 电机抽象                                                        */
/* ========================================================================== */

/**
 *  硬件映射 (修改此处即可适配不同飞控板)
 *
 *  TIM3 通道 → 电机:
 *      CCR1 → M1 (后左 RL, CW)
 *      CCR2 → M2 (前左 FL, CCW)
 *      CCR3 → M3 (后右 RR, CCW)
 *      CCR4 → M4 (前右 FR, CW)
 */
#define MOTOR1_CCR TIM3->CCR1 /* M1: 后左 RL, CW  */
#define MOTOR2_CCR TIM3->CCR2 /* M2: 前左 FL, CCW */
#define MOTOR3_CCR TIM3->CCR3 /* M3: 后右 RR, CCW */
#define MOTOR4_CCR TIM3->CCR4 /* M4: 前右 FR, CW  */

/** @brief 电机索引 */
enum {
    MOTOR_RL = 0, /* M1: 后左 */
    MOTOR_FL = 1, /* M2: 前左 */
    MOTOR_RR = 2, /* M3: 后右 */
    MOTOR_FR = 3, /* M4: 前右 */
    MOTOR_COUNT = 4,
};

/**
 * @brief 写入所有电机 PWM
 * @param m 百分比数组 [M1, M2, M3, M4], 范围 0~100
 */
static inline void Motor_WriteAll(const float m[MOTOR_COUNT]) {
    MOTOR1_CCR = PWM_Map_Percent(m[MOTOR_RL]);
    MOTOR2_CCR = PWM_Map_Percent(m[MOTOR_FL]);
    MOTOR3_CCR = PWM_Map_Percent(m[MOTOR_RR]);
    MOTOR4_CCR = PWM_Map_Percent(m[MOTOR_FR]);
}

/** @brief 停止所有电机 */
static inline void Motor_Stop(void) {
    MOTOR1_CCR = 0;
    MOTOR2_CCR = 0;
    MOTOR3_CCR = 0;
    MOTOR4_CCR = 0;
}

/* ========================================================================== */
/*  Section 2: PID 实例与共享变量                                              */
/* ========================================================================== */

/* 角度环 (外环) — 输出 deg/s */
PID_t pidRoll, pidPitch, pidYaw, pidHeight;

/* 速率环 (内环) — 输出混控指令 */
PID_t pidRateRoll, pidRatePitch, pidRateYaw;

/* 基础 */
uint8_t baseThrottle = 30;

    /* 外环 → 内环的共享变量 (volatile, 中断保护) */
__IO float rateSetRoll = 0.0f;
__IO float rateSetPitch = 0.0f;
__IO float rateSetYaw = 0.0f;
__IO float thrustOutput = 0.0f;

/* ========================================================================== */
/*  Section 3: 内部状态                                                        */
/* ========================================================================== */

static ControlMode_t curMode = CONTROL_MODE_DIRECT;
static FlightPhase_t curPhase = FLIGHT_PHASE_GROUNDED;
static volatile uint8_t isArmed = 0;

static float baseHeight = 0.0f;   /**< 起飞基准高度 (m)       */
static float targetRoll = 0.0f;   /**< 目标 Roll  (°)         */
static float targetPitch = 0.0f;  /**< 目标 Pitch (°)         */
static float targetYaw = 0.0f;    /**< 目标 Yaw   (°)        */
static float targetHeight = 0.0f; /**< 目标高度 (m, 绝对)     */
static float moveForward = 0.0f;  /**< 前进指令 [-1, +1]      */
static float moveRight = 0.0f;    /**< 右移指令 [-1, +1]      */

static float rampedHeight = 0.0f; /**< 高度斜坡当前值 (m)     */
static uint8_t ramp_init = 0;     /**< 斜坡已初始化标志       */

/* 目标四元数 (Hamilton [w,x,y,z]) */
static sm_quat_t targetQuat = {1.0f, 0.0f, 0.0f, 0.0f};

/* ========================================================================== */
/*  Section 4: 内部四元数工具                                                  */
/* ========================================================================== */

/**
 * @brief 四元数乘法 q_out = q_a ⊗ q_b (Hamilton, scalar-first)
 */
static void quat_multiply(sm_quat_t out, const sm_quat_t a, const sm_quat_t b) {
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

/**
 * @brief 四元数共轭 q* = [w, -x, -y, -z]
 */
static inline void quat_conjugate(sm_quat_t q) {
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
}

/**
 * @brief 欧拉角 → 四元数 (ZYX 顺序)
 *
 * @param[out] q        输出四元数 [w,x,y,z]
 * @param[in]  roll_rad  Roll  角 (rad)
 * @param[in]  pitch_rad Pitch 角 (rad)
 * @param[in]  yaw_rad   Yaw   角 (rad)
 */
static void quat_from_euler(sm_quat_t q,
                            float roll_rad,
                            float pitch_rad,
                            float yaw_rad) {
    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cy = cosf(yaw_rad * 0.5f);
    float sy = sinf(yaw_rad * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy; /* w */
    q[1] = sr * cp * cy - cr * sp * sy; /* x */
    q[2] = cr * sp * cy + sr * cp * sy; /* y */
    q[3] = cr * cp * sy - sr * sp * cy; /* z */
}

/**
 * @brief 四元数 → 欧拉角 (ZYX 顺序)
 *
 * @param[in]  q         输入四元数 [w,x,y,z]
 * @param[out] roll_rad  Roll  角 (rad, ±π)
 * @param[out] pitch_rad Pitch 角 (rad, ±π/2)
 * @param[out] yaw_rad   Yaw   角 (rad, ±π)
 */
static void quat_to_euler(const sm_quat_t q,
                          float* roll_rad,
                          float* pitch_rad,
                          float* yaw_rad) {
    float w = q[0], x = q[1], y = q[2], z = q[3];

    /* Roll (X 轴) */
    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    *roll_rad = atan2f(sinr, cosr);

    /* Pitch (Y 轴) — 钳位到 ±π/2 防止 NaN */
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f)
        *pitch_rad = copysignf(M_PI_F * 0.5f, sinp);
    else
        *pitch_rad = asinf(sinp);

    /* Yaw (Z 轴) */
    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);
    *yaw_rad = atan2f(siny, cosy);
}

/* ========================================================================== */
/*  Section 5: 内部工具                                                        */
/* ========================================================================== */

/** @brief 角度归一化到 (-180, 180] */
static float NormalizeAngle(float a) {
    while (a > 180.0f)
        a -= 360.0f;
    while (a <= -180.0f)
        a += 360.0f;
    return a;
}

/** @brief 重置所有 PID 积分和微分状态 */
static void ResetAllPIDs(void) {
    PID_Reset(&pidRoll);
    PID_Reset(&pidPitch);
    PID_Reset(&pidYaw);
    PID_Reset(&pidHeight);
    PID_Reset(&pidRateRoll);
    PID_Reset(&pidRatePitch);
    PID_Reset(&pidRateYaw);
}

/** @brief 重置所有目标到默认值 */
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

/** @brief 安全停止: 清零共享变量和电机 */
static void StopMotors(void) {
    __disable_irq();
    thrustOutput = 0.0f;
    rateSetRoll = 0.0f;
    rateSetPitch = 0.0f;
    rateSetYaw = 0.0f;
    __enable_irq();

    Motor_Stop();
}

/* ========================================================================== */
/*  Section 6: 外环 — 姿态 + 高度控制                                         */
/* ========================================================================== */

/**
 * @brief 外环主函数 (主循环, 200 Hz)
 *
 * 内部流程:
 *   1. 读取当前姿态和高度
 *   2. 飞行阶段状态机 (起飞/降落)
 *   3. 高度环 PID (ALTITUDE 模式)
 *   4. 构建目标四元数 (含平移叠加)
 *   5. 计算误差四元数 → 提取角度误差
 *   6. 角度环 PID → 速率设定点
 */
void ControlAttitude_Loop(void) {
    if (!isArmed) {
        StopMotors();
        return;
    }

    const float dt = 1.0f / (float)ATTITUDE_LOOP_HZ;

    /* ── 1. 读取当前姿态 ─────────────────────────── */
    sm_quat_t curQuat;
    Attitude_GetQuat(curQuat);

    /* NaN 保护 */
    if (!isfinite(curQuat[0]) || !isfinite(curQuat[1]) ||
        !isfinite(curQuat[2]) || !isfinite(curQuat[3])) {
        return;
    }

    /* 当前欧拉角 (rad → deg, 用于日志和状态判断) */
    float curYaw_rad, curPitch_rad, curRoll_rad;
    Attitude_GetEuler(&curYaw_rad, &curPitch_rad, &curRoll_rad);
    float curYaw = curYaw_rad * RAD2DEG;
    float curPitch = curPitch_rad * RAD2DEG;
    float curRoll = curRoll_rad * RAD2DEG;

    float curHeight;
    Attitude_GetAltitude(&curHeight);

    /* ── 2. 降落状态机 ───────────────────────────── */
    if (curPhase == FLIGHT_PHASE_LANDING) {
        if (curHeight < baseHeight + LANDING_HEIGHT_THRESH) {
            curPhase = FLIGHT_PHASE_GROUNDED;
            Control_Disarm();
            return;
        }
    }

    /* ── 3. 起飞状态机 ───────────────────────────── */
    if (curPhase == FLIGHT_PHASE_TAKING_OFF) {
        if (fabsf(curHeight - targetHeight) < TAKEOFF_HEIGHT_THRESH) {
            curPhase = FLIGHT_PHASE_IN_FLIGHT;
        }
    }

    /* ── 4. 高度环 (ALTITUDE 模式) ────────────────── */
    if (curMode >= CONTROL_MODE_ALTITUDE) {
        /* 斜坡初始化 */
        if (!ramp_init) {
            rampedHeight = baseHeight;
            ramp_init = 1;
        }

        /* 斜坡跟踪: 限制目标高度的变化速率 */
        float heightErr = targetHeight - rampedHeight;
        if (heightErr > HEIGHT_RAMP_SPEED * dt)
            rampedHeight += HEIGHT_RAMP_SPEED * dt;
        else if (heightErr < -HEIGHT_RAMP_SPEED * dt)
            rampedHeight -= HEIGHT_RAMP_SPEED * dt;
        else
            rampedHeight = targetHeight;

        /* 高度 PID (前馈: baseThrottle, 反馈: PID 修正) */
        thrustOutput = baseThrottle +
                       PID_Update(&pidHeight, rampedHeight, curHeight, dt);
        thrustOutput = fmaxf(0.0f, fminf(thrustOutput, 100.0f));
    }

    /* ── 5. DIRECT 模式: 不输出角度环 ─────────────── */
    if (curMode == CONTROL_MODE_DIRECT) {
        __disable_irq();
        rateSetRoll = 0.0f;
        rateSetPitch = 0.0f;
        rateSetYaw = 0.0f;
        __enable_irq();
        return;
    }

    /* ── 6. 姿态目标构建 ─────────────────────────── */
    /*
     * 叠加平移倾斜:
     *   前进 → 机头下沉 → Pitch 减小 (FRD 正 pitch = 上仰, 前进需要负 pitch)
     *   右移 → 右翼下沉 → Roll  增大 (FRD 正 roll = 右倾)
     */
    float effRoll = targetRoll + moveRight * TILT_ANGLE_LIMIT;
    float effPitch = targetPitch - moveForward * TILT_ANGLE_LIMIT;
    float effYaw = targetYaw;

    /* 目标四元数 (rad) */
    quat_from_euler(targetQuat,
                    effRoll * DEG2RAD,
                    effPitch * DEG2RAD,
                    effYaw * DEG2RAD);

    /* ── 7. 误差四元数计算 ────────────────────────── */
    /*
     * q_err = q_target^* ⊗ q_current
     *
     * 含义: 从目标到当前的旋转。
     *       若当前 Roll > 目标 Roll → errRoll > 0 (右倾误差)
     *       若当前 Yaw  > 目标 Yaw  → errYaw  > 0 (CW 误差)
     */
    sm_quat_t qTargetInv;
    memcpy(qTargetInv, targetQuat, sizeof(sm_quat_t));
    quat_conjugate(qTargetInv);

    sm_quat_t qErr;
    quat_multiply(qErr, qTargetInv, curQuat);

    /* 最短路径: w < 0 表示走了长弧, 取反 */
    if (qErr[0] < 0.0f) {
        qErr[0] = -qErr[0];
        qErr[1] = -qErr[1];
        qErr[2] = -qErr[2];
        qErr[3] = -qErr[3];
    }

    /* 提取角度误差 (rad → deg) */
    float errRoll_rad, errPitch_rad, errYaw_rad;
    quat_to_euler(qErr, &errRoll_rad, &errPitch_rad, &errYaw_rad);
    float errRoll = errRoll_rad * RAD2DEG;
    float errPitch = errPitch_rad * RAD2DEG;
    float errYaw = errYaw_rad * RAD2DEG;

    /* 万向锁保护: 大俯仰角时 Yaw/Roll 耦合, 禁用 Yaw 修正 */
    float sinPitch = 2.0f * (curQuat[0] * curQuat[2] - curQuat[1] * curQuat[3]);
    if (fabsf(sinPitch) > GIMBAL_LOCK_THRESH) {
        errYaw = 0.0f;
    }

    /* ── 8. 角度环 PID → 速率设定点 ──────────────── */
    /*
     * PID(target=0, measured=errAngle):
     *   error = 0 - errAngle = -errAngle
     *   若 errRoll > 0 (右倾), PID 输出 < 0 (要求左旋速率)
     */
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

/* ========================================================================== */
/*  Section 7: 内环 — 速率控制 + 电机混控                                      */
/* ========================================================================== */

/**
 * @brief 内环主函数 (TIM4 中断, 500 Hz)
 *
 * 流程:
 *   1. 原子读取外环输出的速率设定点
 *   2. 读取陀螺仪实际角速度
 *   3. 速率环 PID → 混控指令
 *   4. 混控器: 油门 + Roll/Pitch/Yaw 分配到四个电机
 *   5. 输出限幅 → 写入 PWM
 */
void ControlMotor_Loop(void) {
    if (!isArmed) {
        Motor_Stop();
        return;
    }

    const float dt = 1.0f / (float)RATE_LOOP_HZ;

    /* ── 1. 原子读取速率设定点 ────────────────────── */
    float localRateSetRoll, localRateSetPitch, localRateSetYaw;
    __disable_irq();
    localRateSetRoll = rateSetRoll;
    localRateSetPitch = rateSetPitch;
    localRateSetYaw = rateSetYaw;
    __enable_irq();

    /* ── 2. 读取陀螺仪 (rad/s → deg/s) ───────────── */
    sm_vec3_t gyro;
    Attitude_GetGyro(gyro); /* bias 已补偿 */

    float gx = gyro[0] * RAD2DEG; /* Roll  rate (deg/s) */
    float gy = gyro[1] * RAD2DEG; /* Pitch rate (deg/s) */
    float gz = gyro[2] * RAD2DEG; /* Yaw   rate (deg/s) */

    /* ── 3. 速率环 PID ───────────────────────────── */
    float rollCtrl = PID_Update(&pidRateRoll, localRateSetRoll, gx, dt);
    float pitchCtrl = PID_Update(&pidRatePitch, localRateSetPitch, gy, dt);
    float yawCtrl = PID_Update(&pidRateYaw, localRateSetYaw, gz, dt);

    float throttle = thrustOutput;

    /* ── 4. 混控器 ────────────────────────────────── */
    /*
     * FRD X 型四旋翼混控:
     *
     *              M1(RL,CW)  M2(FL,CCW)  M3(RR,CCW)  M4(FR,CW)
     * 油门 (↑)      +1          +1          +1          +1
     * Roll  (右倾+) +1          +1          -1          -1
     * Pitch (上仰+) -1          +1          -1          +1
     * Yaw   (CW+)   -1          +1          +1          -1
     *
     * 推导:
     *   Roll  正力矩: 右翼下沉 → 左侧电机推力更大 → M1+, M2+, M3-, M4-
     *   Pitch 正力矩: 机头上抬 → 前方电机推力更大 → M2+, M4+, M1-, M3-
     *   Yaw   正力矩: 机身 CW → CCW 电机推力更大 → M2+, M3+, M1-, M4-
     */
    float m[MOTOR_COUNT];
    m[MOTOR_RL] = throttle + rollCtrl - pitchCtrl - yawCtrl;
    m[MOTOR_FL] = throttle + rollCtrl + pitchCtrl + yawCtrl;
    m[MOTOR_RR] = throttle - rollCtrl - pitchCtrl + yawCtrl;
    m[MOTOR_FR] = throttle - rollCtrl + pitchCtrl - yawCtrl;

    /* ── 5. 输出限幅 ──────────────────────────────── */
    /* 比例限幅: 保持差动指令比例, 避免截断导致姿态失控 */
    float m_min = m[0], m_max = m[0];
    for (int i = 1; i < MOTOR_COUNT; i++) {
        if (m[i] < m_min)
            m_min = m[i];
        if (m[i] > m_max)
            m_max = m[i];
    }

    /* 下溢保护: 将最低电机拉到 0% */
    if (m_min < 0.0f) {
        float shift = -m_min;
        for (int i = 0; i < MOTOR_COUNT; i++)
            m[i] += shift;
    }

    /* 上溢保护: 将最高电机压到 100% */
    if (m_max > 100.0f) {
        float shift = m_max - 100.0f;
        for (int i = 0; i < MOTOR_COUNT; i++)
            m[i] -= shift;
    }

    /* 最终安全钳位 */
    for (int i = 0; i < MOTOR_COUNT; i++)
        m[i] = fmaxf(0.0f, fminf(m[i], 100.0f));

    /* ── 6. 写入电机 ─────────────────────────────── */
    Motor_WriteAll(m);
}

/* ========================================================================== */
/*  Section 8: 初始化                                                          */
/* ========================================================================== */

void Control_Init(void) {
    /* ── TIM4 中断: 内环定时器 ────────────────────── */
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

    /* ── PID 参数 ─────────────────────────────────── */
    /*                     Kp    Ki    Kd    OutMin  OutMax  Df   IntMin  IntMax  Scale */
    /* 角度环 (输出 deg/s) */
    PID_Init(&pidHeight, 10.0f, 1.0f, 6.0f, -15.0f, 15.0f, 0.02f, -40.0f, 40.0f, 1.0f);
    PID_Init(&pidRoll, 4.0f, 0.01f, 0.0f, -20.0f, 20.0f, 0.02f, -100.0f, 100.0f, 1.0f);
    PID_Init(&pidPitch, 4.0f, 0.01f, 0.0f, -20.0f, 20.0f, 0.02f, -100.0f, 100.0f, 1.0f);
    PID_Init(&pidYaw, 2.0f, 0.01f, 0.0f, -20.0f, 20.0f, 0.02f, -100.0f, 100.0f, 1.0f);

    /* 速率环 (输出混控指令) */
    PID_Init(&pidRateRoll, 0.4f, 0.3f, 0.008f, -20.0f, 20.0f, 0.01f, -100.0f, 100.0f, 1.0f);
    PID_Init(&pidRatePitch, 0.4f, 0.3f, 0.008f, -20.0f, 20.0f, 0.01f, -100.0f, 100.0f, 1.0f);
    PID_Init(&pidRateYaw, 0.1f, 0.05f, 0.0f, -20.0f, 20.0f, 0.01f, -100.0f, 100.0f, 1.0f);
}

/* ========================================================================== */
/*  Section 9: 模式切换                                                        */
/* ========================================================================== */

int8_t Control_SetMode(ControlMode_t mode) {
    if (mode > CONTROL_MODE_POSITION)
        return -1;

    curMode = mode;
    moveForward = 0.0f;
    moveRight = 0.0f;
    ResetAllPIDs();

    return (int8_t)mode;
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
    memcpy(out, targetQuat, sizeof(sm_quat_t));
}

/* ========================================================================== */
/*  Section 10: 解锁 / 锁定                                                   */
/* ========================================================================== */

int8_t Control_Arm(void) {
    if (curMode != CONTROL_MODE_DIRECT)
        return -1;
    if (thrustOutput > 1.0f)
        return -1;

    StopMotors();
    ResetAllPIDs();

    /*
     * 先采集当前状态，再重置目标:
     *   baseHeight  ← 当前高度 (供 ResetAllTargets 中 targetHeight 使用)
     *   currentYaw  ← 当前航向 (解锁后保持此朝向)
     */
    Attitude_GetAltitude(&baseHeight);

    float curYawRad, curPitchRad, curRollRad;
    Attitude_GetEuler(&curYawRad, &curPitchRad, &curRollRad);

    ResetAllTargets();                               /* targetHeight = baseHeight */
    targetYaw = NormalizeAngle(curYawRad * RAD2DEG); /* 保持当前航向 */

    curPhase = FLIGHT_PHASE_GROUNDED;
    isArmed = 1;

    return (int8_t)curPhase;
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

/* ========================================================================== */
/*  Section 11: 飞行操作                                                       */
/* ========================================================================== */

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

    /* 自动提升到 ALTITUDE 模式 */
    if (curMode < CONTROL_MODE_ALTITUDE)
        Control_SetMode(CONTROL_MODE_ALTITUDE);

    curPhase = FLIGHT_PHASE_TAKING_OFF;

    return (int8_t)curPhase;
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

    return (int8_t)curPhase;
}

void Control_Hover(void) {
    if (curMode < CONTROL_MODE_ALTITUDE)
        return;

    moveForward = 0.0f;
    moveRight = 0.0f;
    targetRoll = 0.0f;
    targetPitch = 0.0f;
}

/* ========================================================================== */
/*  Section 12: 指令输入                                                       */
/* ========================================================================== */

void Control_SetThrottle(float throttle) {
    if (!isArmed) {
        thrustOutput = 0.0f;
        return;
    }

    /* 仅 DIRECT 和 STABILIZED 模式接受手动油门 */
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
