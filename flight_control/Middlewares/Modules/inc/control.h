/**
 * @file control.h
 * @brief 四旋翼飞行控制模块 — 公共接口
 *
 * ============================================================================
 *  坐标系与姿态约定 (与 attitude / ekf 保持一致)
 * ============================================================================
 *
 *  世界坐标系: NED  (North-East-Down)
 *  机体坐标系: FRD  (Forward-Right-Down)
 *
 *  欧拉角旋转顺序: ZYX  (Yaw → Pitch → Roll), 单位: 度 (°)
 *      Roll  (φ): 右倾为正     Pitch (θ): 低头为正     Yaw (ψ): 机头右偏为正
 *
 *  四元数: Hamilton 约定, 标量在前 [w, x, y, z]
 *
 * ============================================================================
 *  电机排布 (X 型, FRD 机体系, 俯视图)
 * ============================================================================
 *
 *              机头 (+X_b)
 *                ↑
 *        M2(FL) ─┼─ M4(FR)
 *           ╲    │    ╱
 *            ╲   │   ╱
 *  左(-Y_b) ── ╳ ── 右(+Y_b)
 *            ╱   │   ╲
 *           ╱    │    ╲
 *        M1(RL) ─┼─ M3(RR)
 *                ↓
 *              机尾 (-X_b)
 *
 *      M1   后左 (Rear-Left)    CW  顺时针
 *      M2   前左 (Front-Left)   CCW 逆时针
 *      M3   后右 (Rear-Right)   CCW 逆时针
 *      M4   前右 (Front-Right)  CW  顺时针
 *
 *      对角电机转向相同: {M1(CW), M4(CW)}  {M2(CCW), M3(CCW)}
 *
 * ============================================================================
 *  控制架构
 * ============================================================================
 *
 *      ┌──────────────┐    rateSetpoint    ┌──────────────┐    motorCmd
 *      │  外环 (姿态)  │ ───────────────→  │  内环 (速率)  │ ──────────→ 混控器
 *      │  主循环同帧   │   deg/s           │  同帧顺序执行  │   (deg/s)
 *      └──────────────┘                    └──────────────┘
 *
 *  外环: 目标角度 vs 当前角度 → PID → 速率设定点 (deg/s)
 *  内环: 速率设定点 vs 陀螺仪  → PID → 电机混控指令
 *  同线程: 主循环同帧顺序调用, 共享变量 volatile + __disable_irq 兑底
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "attitude.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*  环路频率                                                                   */
/* ========================================================================== */

#define ATTITUDE_LOOP_HZ 200 /**< 外环标称 (主循环 5ms 帧); 正常用 FPS 实测 dt, 此值仅兑底 */
#define RATE_LOOP_HZ 200     /**< 内环与外环同帧执行, 实测 dt */

/* ========================================================================== */
/*  控制模式                                                                   */
/* ========================================================================== */

/**
 * @brief 控制模式 (从低到高递进)
 *
 *  DIRECT     : 直接油门, 无姿态稳定
 *  STABILIZED : 姿态稳定 (角度环 + 速率环), 手动油门
 *  ALTITUDE   : 高度保持 + 姿态稳定
 *  POSITION   : 位置保持 (预留, 未实现)
 */
typedef enum {
    CONTROL_MODE_DIRECT = 0,
    CONTROL_MODE_STABILIZED = 1,
    CONTROL_MODE_ALTITUDE = 2,
    CONTROL_MODE_POSITION = 3,
} ControlMode_t;

/**
 * @brief 飞行阶段
 */
typedef enum {
    FLIGHT_PHASE_GROUNDED = 0,   /**< 地面锁定               */
    FLIGHT_PHASE_TAKING_OFF = 1, /**< 起飞中 (爬升到目标高度) */
    FLIGHT_PHASE_IN_FLIGHT = 2,  /**< 悬停/飞行中             */
    FLIGHT_PHASE_LANDING = 3,    /**< 降落中 (下降到地面)     */
} FlightPhase_t;

/* ========================================================================== */
/*  PID 实例 (外部可见, 便于调参工具访问)                                       */
/* ========================================================================== */

/** @name 角度环 PID (外环, 输出 deg/s) */
/** @{ */
extern PID_t pidRoll;   /**< Roll  角度环 PID  */
extern PID_t pidPitch;  /**< Pitch 角度环 PID  */
extern PID_t pidYaw;    /**< Yaw   角度环 PID  */
extern PID_t pidHeight; /**< 高度环 PID        */
/** @} */

/** @name 速率环 PID (内环, 输出混控指令) */
/** @{ */
extern PID_t pidRateRoll;  /**< Roll  速率环 PID  */
extern PID_t pidRatePitch; /**< Pitch 速率环 PID  */
extern PID_t pidRateYaw;   /**< Yaw   速率环 PID  */
/** @} */

/* ========================================================================== */
/*  共享变量 (外环写入, 内环读取, 中断保护)                                     */
/* ========================================================================== */

extern __IO float rateSetRoll;  /**< Roll  速率设定点 (deg/s) */
extern __IO float rateSetPitch; /**< Pitch 速率设定点 (deg/s) */
extern __IO float rateSetYaw;   /**< Yaw   速率设定点 (deg/s) */
extern __IO float thrustOutput; /**< 油门输出 (0~100%)        */
extern __IO float rateCmdRoll;  /**< Roll 速率环输出 (混控前, 诊断遥测 0x04) */

/* ========================================================================== */
/*  生命周期                                                                   */
/* ========================================================================== */

/** @brief 初始化控制模块 (PID 参数、定时器、中断) */
void Control_Init(void);

/* ========================================================================== */
/*  环路函数                                                                   */
/* ========================================================================== */

/**
 * @brief 外环 — 姿态 + 高度控制
 * @note  由主循环以 ATTITUDE_LOOP_HZ 频率调用
 *
 * 流程: 读姿态 → 解析指令 → 角度环 PID → 写入速率设定点
 */
void ControlAttitude_Loop(void);

/**
 * @brief 内环 — 速率控制 + 电机混控
 * @note  与外环同帧在主循环顺序调用 (不再挂 TIM4), 用实测 dt
 *
 * 流程: 读陀螺仪 → 速率环 PID → 混控 → 写入电机 PWM
 */
void ControlMotor_Loop(void);

/* ========================================================================== */
/*  模式 / 状态查询                                                            */
/* ========================================================================== */

/** @brief 设置控制模式, 返回实际生效模式 (失败返回 -1) */
int8_t Control_SetMode(ControlMode_t mode);

/** @brief 获取当前控制模式 */
ControlMode_t Control_GetMode(void);

/** @brief 获取当前飞行阶段 */
FlightPhase_t Control_GetFlightPhase(void);

/** @brief 获取起飞基准高度 (m) */
float Control_GetBaseHeight(void);

/** @brief 获取当前目标四元数 [w,x,y,z] */
void Control_GetTargetQuat(sm_quat_t out);

/** @brief 是否已解锁 */
uint8_t Control_IsArmed(void);

/* ========================================================================== */
/*  解锁 / 锁定                                                               */
/* ========================================================================== */

/** @brief 解锁 (仅 DIRECT 模式且油门为零时允许), 返回飞行阶段 */
int8_t Control_Arm(void);

/** @brief 锁定 (停电机、重置 PID) */
int8_t Control_Disarm(void);

/** @brief 紧急停止 (立即锁定, 忽略状态检查) */
void Control_EmergencyStop(void);

/**
 * @brief Roll 开环阶跃测试 (诊断): 固定差动输出 N 帧, 旁路速率环
 * @param cmd    混控差动指令 (建议 5~10 起步, 内部钳位 ±20)
 * @param frames 持续帧数 (0 = 提前结束; 上限 400 帧 = 2s @200Hz)
 * @return 0=已启动/已取消, -1=未解锁 (电机未转, 测了也白测)
 * @note  配合遥测 0x04 (rollCmd 阶跃 vs gx 响应) 读执行机构延迟、
 *        上升时间与死区; 结束后自动复位 roll 角度/速率 PID。
 *        测试期间飞行器会开环滚转 — 手持/系留 + 小幅度短时！
 */
int8_t Control_RollOpenLoopTest(float cmd, uint16_t frames);

/** @brief 摆动周期 (ms): 滤波后 gx 过零测得; >2s 无过零返回 0 (未在摆) */
uint16_t Control_GetOscPeriodMs(void);
/** @brief 摆幅: errRoll 峰峰值 (0.1° 单位), 统计窗口 ≥1.5s 自动重开 */
uint16_t Control_GetOscAmpDeciDeg(void);
/** @brief 电机是否触轨 (0/100 鉳位发生过); 读取即清零 */
uint8_t Control_TakeMotorSat(void);

/* ========================================================================== */
/*  飞行操作                                                                   */
/* ========================================================================== */

/**
 * @brief 起飞
 * @param relative_height 相对基准高度的爬升高度 (m, > 0.1)
 * @return 飞行阶段, 失败返回 -1
 */
int8_t Control_Takeoff(float relative_height);

/** @brief 降落 (缓慢下降到基准高度后自动锁定) */
int8_t Control_Land(void);

/** @brief 悬停 (清除平移指令, 保持当前高度和航向) */
void Control_Hover(void);

/* ========================================================================== */
/*  指令输入                                                                   */
/* ========================================================================== */

/**
 * @brief 设置油门 (仅 DIRECT / STABILIZED 模式有效)
 * @param throttle 0~100%
 */
void Control_SetThrottle(float throttle);

/**
 * @brief 设置目标姿态
 * @param roll  目标 Roll  角 (°, ±45)
 * @param pitch 目标 Pitch 角 (°, ±45)
 * @param yaw   目标 Yaw   角 (°, 自动归一化到 ±180)
 */
void Control_SetAttitude(float roll, float pitch, float yaw);

/**
 * @brief 设置平移指令 (仅 ALTITUDE 及以上模式有效)
 * @param forward -1~+1 (正 = 前进)
 * @param right   -1~+1 (正 = 右移)
 */
void Control_Move(float forward, float right);

/**
 * @brief 设置目标高度 (仅 ALTITUDE 及以上模式有效)
 * @param height 绝对高度 (m, 不低于基准高度)
 */
void Control_SetHeight(float height);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
