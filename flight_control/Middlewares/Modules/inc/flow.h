#ifndef __FLOW_H
#define __FLOW_H

#include "stm32f4xx.h"

/* ── 光流子板 (STM32G031 + PMW3901 + VL53L1X) 二进制协议 ──────────────
 * 36 字节定长帧, 小端:
 *   [0]     0xAA
 *   [1]     0x55
 *   [2]     len = 33 (覆盖偏移 3..35)
 *   [3]     seq (自增回绕)
 *   [4..11] time_us        (u64, 上电起 µs)
 *   [12..15] integration_us(u32, µs)
 *   [16..19] flow_x_rad    (f32, 纯角度, 未乘高度)
 *   [20..23] flow_y_rad    (f32, 纯角度, 未乘高度)
 *   [24..27] distance_m    (f32, m, <0 = 未知/无效)
 *   [28]    quality        (0..255)
 *   [29]    status         (RangeStatus, 255 = 激光失联)
 *   [30]    valid          (0=无位移 1=有效 2=被拒)
 *   [31..32] raw_dx        (i16, 像素)
 *   [33..34] raw_dy        (i16, 像素)
 *   [35]    crc8           (覆盖偏移 2..34, 共 33 字节)
 *
 * 关键语义:
 *   - flow_x/y_rad 是图像旋转角, 线位移 = 角度(rad) × distance_m
 *   - PX4 轴向: 机体向前 -> flow_y_rad 为正; 机体向右 -> flow_x_rad 为负
 *   - distance_m < 0 表示距离未知 (0 是合法近距离值), 此时冻结积分
 *   - status == 255 表示激光失联
 */

#define FLOW_FRAME_SIZE         36u
#define FLOW_SYNC1              0xAAu
#define FLOW_SYNC2              0x55u
#define FLOW_PAYLOAD_LEN        33u   /* len 字段值, CRC 覆盖 33 字节 */
#define FLOW_CRC_OFFSET         35u
#define FLOW_TIMEOUT_MS         200u  /* 超过 200ms 无有效帧 -> 链路断开 */
#define FLOW_STATUS_LASER_LOST  255u

/* 显示方向修正 (只影响航位推算累加方向, 不改协议原始值):
 * 按 PX4 轴向约定, 屏幕 +X 向右 / +Y 向下, 二者都取 -1 时
 * "前进" 在屏幕上向上、"右移" 在屏幕上向右。
 * 实测方向相反就把对应宏改成 +1.0f。 */
#ifndef FLOW_DISP_X_SIGN
#define FLOW_DISP_X_SIGN        (-1.0f)
#endif
#ifndef FLOW_DISP_Y_SIGN
#define FLOW_DISP_Y_SIGN        (-1.0f)
#endif

/* 累积位置硬限幅 (m): 防止长时间运行 float 漂移/溢出 */
#define FLOW_POS_LIMIT_M        50.0f

typedef struct {
    /* ── 子板上报字段 ── */
    uint8_t  seq;
    uint64_t time_us;
    uint32_t integration_us;
    float    flow_x_rad;
    float    flow_y_rad;
    float    distance_m;
    uint8_t  quality;
    uint8_t  status;
    uint8_t  valid;
    int16_t  raw_dx;
    int16_t  raw_dy;

    /* ── 主控侧计算/统计 ── */
    uint32_t rx_ok;         /* 有效帧数 (CRC 通过) */
    uint32_t rx_crc_err;    /* CRC/长度错误帧数 */
    uint32_t rx_lost;       /* 用 seq 差值估算的丢帧数 */
    uint8_t  link_ok;       /* 1 = 200ms 内收到过有效帧 */
    uint32_t last_rx_tick;  /* 最近有效帧的 systemTick (ms) */
} flow_data_t;

void Flow_Init(void);

/* 字节流喂入: 内部为 AA 55 同步头状态机, 任意长度分片都可以 */
void Flow_FeedByte(uint8_t byte);
void Flow_FeedBytes(const uint8_t *data, uint16_t len);

/* 主循环周期调用: 链路超时检测 */
void Flow_Update(void);

const flow_data_t *Flow_Get(void);
uint8_t Flow_LinkOk(void);

/* 航位推算累积位置 (m), 供 WINDOW_FLOW 显示 */
void  Flow_ResetPosition(void);
float Flow_GetPosX(void);
float Flow_GetPosY(void);

/* CRC-8: poly 0x07, init 0x00, 无反射, 无终值异或 */
uint8_t Flow_Crc8(const uint8_t *data, uint16_t len);

#endif /* __FLOW_H */
