#ifndef __COMMAND_H
#define __COMMAND_H

#include "stm32f4xx.h"

typedef enum {
    /* UI 指令 0x00 ~ 0x0F */
    CMD_TYPE_UI_CLICK              = 0x00,
    CMD_TYPE_UI_UP                 = 0x01,
    CMD_TYPE_UI_DOWN               = 0x02,
    CMD_TYPE_UI_LEFT               = 0x03,
    CMD_TYPE_UI_RIGHT              = 0x04,

    /* 控制级 0x10 ~ 0x1F */
    CMD_TYPE_CONTROL_MODE         = 0x10,   /* 切换控制层级      */
    CMD_TYPE_CONTROL_THROTTLE     = 0x11,   /* 直接油门         */
    CMD_TYPE_CONTROL_HEIGHT       = 0x12,   /* 设目标高度        */
    CMD_TYPE_CONTROL_MOVE         = 0x13,   /* 平移指令         */
    CMD_TYPE_CONTROL_ATTITUDE     = 0x14,   /* 设目标姿态        */
    CMD_TYPE_CONTROL_ARM          = 0x15,   /* 解锁             */
    CMD_TYPE_CONTROL_DISARM       = 0x16,   /* 锁定             */
    CMD_TYPE_CONTROL_ESTOP        = 0x17,   /* 紧急停止         */
    CMD_TYPE_CONTROL_TAKEOFF      = 0x18,   /* 起飞（带高度参数）  */
    CMD_TYPE_CONTROL_LAND         = 0x19,   /* 降落             */
    CMD_TYPE_CONTROL_HOVER        = 0x1A,   /* 悬停             */
} CommandType_t;


/**
 * @brief  解析并执行一条命令
 * @param  data  命令帧（首字节为 CommandType_t，后续为参数）
 * @param  len   帧长度
 * @return 0=成功, -1=未知命令, -2=参数错误
 */
uint8_t Command_ParseAndExecute(const uint8_t* data, uint16_t len);


/** @brief  注册：切换控制层级 (ControlMode_t) → int8_t */
void Command_SetModeCallback(int8_t (*callback)(uint8_t new_mode));
/** @brief  注册：直接油门 (float) → void */
void Command_SetThrottleCallback(void (*callback)(float throttle));
/** @brief  注册：设目标高度 (float) → void */
void Command_SetHeightCallback(void (*callback)(float height));
/** @brief  注册：平移指令 (float, float) → void */
void Command_MoveCallback(void (*callback)(float forward, float right));
/** @brief  注册：设目标姿态 (float, float, float) → void */
void Command_SetAttitudeCallback(void (*callback)(float roll, float pitch, float yaw));
/** @brief  注册：解锁 () → int8_t */
void Command_SetArmCallback(int8_t (*callback)(void));
/** @brief  注册：锁定 () → int8_t */
void Command_SetDisarmCallback(int8_t (*callback)(void));
/** @brief  注册：紧急停止 () → void */
void Command_SetEStopCallback(void (*callback)(void));
/** @brief  注册：起飞 (float) → int8_t */
void Command_SetTakeoffCallback(int8_t (*callback)(float relative_height));
/** @brief  注册：降落 () → int8_t */
void Command_SetLandCallback(int8_t (*callback)(void));
/** @brief  注册：悬停 () → void */
void Command_SetHoverCallback(void (*callback)(void));

#endif /* __COMMAND_H */