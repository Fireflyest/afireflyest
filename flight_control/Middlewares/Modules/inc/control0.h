#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx.h"
#include "pid.h"
#include "lowpass.h"
#include "spatial_math.h"

/* ══════════════════════════════════════════════════════════════
 *  模块说明
 * ══════════════════════════════════════════════════════════════
 *
 *  控制架构：单一控制层级 + 内部飞行阶段状态机
 *
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  ControlMode (控制层级)                                   │
 *  │    用户设置，决定"摇杆输入被解释为什么"                    │
 *  │                                                          │
 *  │    DIRECT ──► STABILIZED ──► ALTITUDE ──► VELOCITY ──► POSITION │
 *  │    全手动      姿态自动       高度自动      速度指令      位置指令  │
 *  └──────────────────────────────────────────────────────────┘
 *
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  FlightPhase (飞行阶段) — 内部状态机，只读                │
 *  │    由 Takeoff / Land / 传感器 / 接地检测 自动驱动         │
 *  │                                                          │
 *  │    GROUNDED ──► TAKING_OFF ──► IN_FLIGHT ──► LANDING ──► GROUNDED │
 *  └──────────────────────────────────────────────────────────┘
 *
 *  典型用法：
 *    Control_Arm();                    // 在 DIRECT 模式下解锁
 *    Control_Takeoff(1.0f);            // 起飞：当前高度 + 1m，自动切 ALTITUDE
 *    Control_Move(0.5f, 0.0f);         // 前进
 *    Control_Land();                   // 降落：高度渐降至 0，到位自动 Disarm
 *
 * ══════════════════════════════════════════════════════════════ */

/* ──────────────────────────────────────────────────────────────
 *  外部变量（供 motor loop 等模块访问）
 * ────────────────────────────────────────────────────────────── */

extern PID_t pidRoll, pidPitch, pidYaw, pidHeight;
extern PID_t pidRateRoll, pidRatePitch, pidRateYaw;
extern float baseThrottle;
extern __IO float rateSetRoll, rateSetPitch, rateSetYaw;
extern __IO float thrustOutput;

/* ──────────────────────────────────────────────────────────────
 *  常量
 * ────────────────────────────────────────────────────────────── */

#define RATE_LOOP_HZ     1000
#define ATTITUDE_LOOP_HZ 200
#define GYRO_TAU         0.004f
#define DEG2RAD          0.017453292519943296f
#define RAD2DEG          57.29577951308232f

/* ──────────────────────────────────────────────────────────────
 *  控制层级 — 单一枚举，按抽象程度递增
 *
 *  ┌────────────┬──────┬──────┬──────┬──────────┐
 *  │   模式     │ 油门 │ 姿态 │ 高度 │   平移    │
 *  ├────────────┼──────┼──────┼──────┼──────────┤
 *  │ DIRECT     │ 手动 │ 手动 │  —   │    —     │
 *  │ STABILIZED │ 手动 │ 自动 │  —   │    —     │
 *  │ ALTITUDE   │ 自动 │ 自动 │ 自动 │ 倾斜映射  │
 *  │ VELOCITY   │ 自动 │ 自动 │ 自动 │ 速度指令  │
 *  │ POSITION   │ 自动 │ 自动 │ 自动 │ 位置指令  │
 *  └────────────┴──────┴──────┴──────┴──────────┘
 * ────────────────────────────────────────────────────────────── */

typedef enum {
    CONTROL_MODE_DIRECT     = 0,    /* 全手动：油门 + 姿态速率 */
    CONTROL_MODE_STABILIZED = 1,    /* 自稳：  油门手动，姿态角度闭环 */
    CONTROL_MODE_ALTITUDE   = 2,    /* 定高：  高度闭环，倾斜映射平移 */
    CONTROL_MODE_VELOCITY   = 3,    /* 速度：  速度闭环 */
    CONTROL_MODE_POSITION   = 4,    /* 位置：  位置闭环 */
} ControlMode_t;

/* ──────────────────────────────────────────────────────────────
 *  飞行阶段 — 内部状态机，由 Takeoff/Land/传感器自动驱动
 *
 *  用户不可直接设置，仅通过 Control_GetFlightPhase() 查询
 *
 *  状态转换：
 *    GROUNDED ──Takeoff()──► TAKING_OFF ──高度达标──► IN_FLIGHT
 *    IN_FLIGHT ──Land()────► LANDING ────接地+高度0──► GROUNDED
 *    任意 ──────E-Stop()───► GROUNDED
 * ────────────────────────────────────────────────────────────── */

typedef enum {
    FLIGHT_PHASE_GROUNDED    = 0,   /* 地面待机（已 Disarm 或待解锁） */
    FLIGHT_PHASE_TAKING_OFF  = 1,   /* 正在起飞，爬升到目标高度 */
    FLIGHT_PHASE_IN_FLIGHT   = 2,   /* 飞行中（含悬停） */
    FLIGHT_PHASE_LANDING     = 3,   /* 正在降落 */
} FlightPhase_t;

/* ──────────────────────────────────────────────────────────────
 *  基础控制
 * ────────────────────────────────────────────────────────────── */

void    Control_Init(void);     /* 初始化 PID 参数等，freq=控制循环频率 */
void    ControlAttitude_Loop(void);                         /* 外环：角度 + 高度 (200 Hz) */
void    ControlMotor_Loop(void);                            /* 内环：速率    (1000 Hz)    */

/* ──────────────────────────────────────────────────────────────
 *  模式查询与切换
 * ────────────────────────────────────────────────────────────── */

/**
 * @brief  切换控制层级
 * @param  mode  CONTROL_MODE_DIRECT ~ POSITION
 * @return 0=成功, -1=传感器不满足该层级要求（保持当前模式不变）
 *
 * @note   切换时自动复位全部 PID 积分项
 */
int8_t  Control_SetMode(ControlMode_t mode);

/** @brief  获取当前控制层级 */
ControlMode_t Control_GetMode(void);

/** @brief  获取当前飞行阶段（只读） */
FlightPhase_t Control_GetFlightPhase(void);

/** @brief  获取上电时记录的基准高度 (m) */
float   Control_GetBaseHeight(void);

/* ──────────────────────────────────────────────────────────────
    *  解锁 / 锁定
    * ────────────────────────────────────────────────────────────── */

/**
 * @brief  解锁电机
 * @return 0=成功, -1=条件不满足
 *
 * @note   安全条件：必须在 DIRECT 模式 + 油门 = 0
 *         解锁时记录上电基准高度，FlightPhase 设为 GROUNDED
 */
int8_t Control_Arm(void);

uint8_t Control_IsArmed(void);

/**
 * @brief  锁定电机（仅在地面 GROUNDED 状态允许）
 * @return 0=成功, -1=仍在飞行中（请先 Land 或 E-Stop）
 */
int8_t  Control_Disarm(void);

/**
 * @brief  紧急停止 — 无条件切断电机
 *
 * @note   最高优先级，不做任何条件检查
 *         建议绑定硬件失控检测中断
 *         立即：电机归零、锁电机、切 DIRECT、FlightPhase → GROUNDED
 */
void    Control_EmergencyStop(void);

/* ──────────────────────────────────────────────────────────────
 *  飞行操作（驱动飞行阶段状态机）
 * ────────────────────────────────────────────────────────────── */

/**
 * @brief  起飞
 * @param  relative_height  相对上电基准高度的上升量 (m)，≥ 0.1
 * @return 0=成功, -1=未解锁或已在飞行中
 *
 * @note   实际目标高度 = 上电基准高度 + relative_height
 *         自动将控制层级提升到 ALTITUDE（如当前更低）
 *         FlightPhase: GROUNDED → TAKING_OFF
 */
int8_t  Control_Takeoff(float relative_height);

/**
 * @brief  降落
 * @return 0=成功, -1=未在飞行中
 *
 * @note   目标高度渐降至上电基准高度
 *         FlightPhase: IN_FLIGHT → LANDING
 *         接地检测通过后自动 Disarm，FlightPhase → GROUNDED
 */
int8_t  Control_Land(void);

/**
 * @brief  悬停 — 清零平移指令，保持当前高度和位置
 *
 * @note   仅在 ALTITUDE 及以上层级生效
 */
void    Control_Hover(void);

/* ──────────────────────────────────────────────────────────────
 *  指令输入（供遥控器 / 上位机调用）
 * ────────────────────────────────────────────────────────────── */

/**
 * @brief  直接设置油门百分比
 * @param  throttle  0 ~ 100
 *
 * @note   仅在 DIRECT / STABILIZED 模式生效
 *         其他模式下油门由高度环自动管理
 *         < 2% 视为零油门（死区）
 */
void    Control_SetThrottle(float throttle);

/**
 * @brief  设置目标姿态角
 * @param  roll   目标横滚 (°)，限制 ±45
 * @param  pitch  目标俯仰 (°)，限制 ±45
 * @param  yaw    目标偏航 (°)，自动归一化到 (-180, 180]
 *
 * @note   STABILIZED 及以上层级生效
 */
void    Control_SetAttitude(float roll, float pitch, float yaw);

/**
 * @brief  设置平面移动指令
 * @param  forward  前进 [-1, 1]（正值 = 前进）
 * @param  right    右移 [-1, 1]（正值 = 右移）
 *
 * @note   ALTITUDE 模式：叠加到目标俯仰/横滚（倾斜映射）
 *         VELOCITY 模式：作为速度指令
 *         DIRECT / STABILIZED 模式：忽略
 */
void    Control_Move(float forward, float right);

/**
 * @brief  设置目标绝对高度
 * @param  height  目标高度 (m)，≥ 上电基准高度
 *
 * @note   ALTITUDE 及以上层级生效
 *         通常不需要手动调用，Takeoff/Land 会自动管理
 */
void    Control_SetHeight(float height);

#endif /* __CONTROL_H */
