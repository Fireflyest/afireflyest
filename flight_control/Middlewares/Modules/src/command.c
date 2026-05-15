#include "command.h"
#include <string.h>

/* ══════════════════════════════════════════════════════════════
 *  回调存储
 * ══════════════════════════════════════════════════════════════ */

static int8_t  (*cbMode)(uint8_t)                 = NULL;
static void    (*cbThrottle)(float)               = NULL;
static void    (*cbHeight)(float)                 = NULL;
static void    (*cbMove)(float, float)            = NULL;
static void    (*cbAttitude)(float, float, float) = NULL;
static int8_t  (*cbArm)(void)                     = NULL;
static int8_t  (*cbDisarm)(void)                  = NULL;
static void    (*cbEStop)(void)                   = NULL;
static int8_t  (*cbTakeoff)(float)                = NULL;
static int8_t  (*cbLand)(void)                    = NULL;
static void    (*cbHover)(void)                   = NULL;

/* ══════════════════════════════════════════════════════════════
 *  回调注册
 * ══════════════════════════════════════════════════════════════ */

void Command_SetModeCallback(int8_t (*cb)(uint8_t))
{
    cbMode = cb;
}

void Command_SetThrottleCallback(void (*cb)(float))
{
    cbThrottle = cb;
}

void Command_SetHeightCallback(void (*cb)(float))
{
    cbHeight = cb;
}

void Command_MoveCallback(void (*cb)(float, float))
{
    cbMove = cb;
}

void Command_SetAttitudeCallback(void (*cb)(float, float, float))
{
    cbAttitude = cb;
}

void Command_SetArmCallback(int8_t (*cb)(void))
{
    cbArm = cb;
}

void Command_SetDisarmCallback(int8_t (*cb)(void))
{
    cbDisarm = cb;
}

void Command_SetEStopCallback(void (*cb)(void))
{
    cbEStop = cb;
}

void Command_SetTakeoffCallback(int8_t (*cb)(float))
{
    cbTakeoff = cb;
}

void Command_SetLandCallback(int8_t (*cb)(void))
{
    cbLand = cb;
}

void Command_SetHoverCallback(void (*cb)(void))
{
    cbHover = cb;
}

/* ══════════════════════════════════════════════════════════════
 *  小端序 float 读取（安全，不依赖对齐）
 * ══════════════════════════════════════════════════════════════ */

static float ReadFloat(const uint8_t* p)
{
    float f;
    memcpy(&f, p, sizeof(float));
    return f;
}

/* ══════════════════════════════════════════════════════════════
 *  解析并执行
 *
 *  输入格式：[TYPE: 1byte] [DATA: 0~N bytes]
 *
 *  各指令数据布局：
 *    0x10  [mode: u8]                          总长 2
 *    0x11  [throttle: float]                   总长 5
 *    0x12  [height: float]                     总长 5
 *    0x13  [forward: float] [right: float]     总长 9
 *    0x14  [roll: float] [pitch: float]
 *          [yaw: float]                        总长 13
 *    0x15  (无数据)                             总长 1
 *    0x16  (无数据)                             总长 1
 *    0x17  (无数据)                             总长 1
 *    0x18  [height: float]                     总长 5
 *    0x19  (无数据)                             总长 1
 *    0x1A  (无数据)                             总长 1
 * ══════════════════════════════════════════════════════════════ */

uint8_t Command_ParseAndExecute(const uint8_t* data, uint16_t len)
{
    if (len < 1) return 0xFF;

    uint8_t type = data[0];
    const uint8_t* payload = data + 1;
    uint16_t payloadLen = len - 1;

    switch (type) {

    /* ── 切换控制层级: [mode: u8] ── */
    case CMD_TYPE_CONTROL_MODE:
        if (payloadLen >= 1 && cbMode) {
            return (uint8_t)cbMode(payload[0]);
        }
        break;

    /* ── 油门: [throttle: float] ── */
    case CMD_TYPE_CONTROL_THROTTLE:
        if (payloadLen >= 4 && cbThrottle) {
            cbThrottle(ReadFloat(payload));
            return 0;
        }
        break;

    /* ── 目标高度: [height: float] ── */
    case CMD_TYPE_CONTROL_HEIGHT:
        if (payloadLen >= 4 && cbHeight) {
            cbHeight(ReadFloat(payload));
            return 0;
        }
        break;

    /* ── 平移: [forward: float] [right: float] ── */
    case CMD_TYPE_CONTROL_MOVE:
        if (payloadLen >= 8 && cbMove) {
            cbMove(ReadFloat(payload), ReadFloat(payload + 4));
            return 0;
        }
        break;

    /* ── 姿态: [roll: float] [pitch: float] [yaw: float] ── */
    case CMD_TYPE_CONTROL_ATTITUDE:
        if (payloadLen >= 12 && cbAttitude) {
            cbAttitude(ReadFloat(payload),
                       ReadFloat(payload + 4),
                       ReadFloat(payload + 8));
            return 0;
        }
        break;

    /* ── 解锁: 无数据 ── */
    case CMD_TYPE_CONTROL_ARM:
        if (cbArm) return (uint8_t)cbArm();
        break;

    /* ── 锁定: 无数据 ── */
    case CMD_TYPE_CONTROL_DISARM:
        if (cbDisarm) return (uint8_t)cbDisarm();
        break;

    /* ── 紧急停止: 无数据 ── */
    case CMD_TYPE_CONTROL_ESTOP:
        if (cbEStop) {
            cbEStop();
            return 0;
        }
        break;

    /* ── 起飞: [relative_height: float] ── */
    case CMD_TYPE_CONTROL_TAKEOFF:
        if (payloadLen >= 4 && cbTakeoff) {
            return (uint8_t)cbTakeoff(ReadFloat(payload));
        }
        break;

    /* ── 降落: 无数据 ── */
    case CMD_TYPE_CONTROL_LAND:
        if (cbLand) return (uint8_t)cbLand();
        break;

    /* ── 悬停: 无数据 ── */
    case CMD_TYPE_CONTROL_HOVER:
        if (cbHover) {
            cbHover();
            return 0;
        }
        break;

    default:
        break;
    }

    return 0xFF;  /* 未注册回调或参数不足 */
}
