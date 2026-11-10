#include "flow.h"
#include "fps.h"

#include <math.h>
#include <string.h>

/* 解析状态 */
#define FLOW_RX_SYNC1   0u  /* 搜索 0xAA */
#define FLOW_RX_SYNC2   1u  /* 已收到 0xAA, 等 0x55 */
#define FLOW_RX_PAYLOAD 2u  /* 正在收集 36 字节 */

static flow_data_t s_flow;
static uint8_t  s_rxBuf[FLOW_FRAME_SIZE];
static uint16_t s_rxIdx;
static uint8_t  s_rxState;
static uint8_t  s_haveSeq;
static uint8_t  s_lastSeq;

/* 航位推算累积位置 (已含显示方向修正), 单位 m */
static float s_posX;
static float s_posY;

/* ── 小端解码 ─────────────────────────────────────────────────────── */

static uint32_t flow_rd_u32(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint64_t flow_rd_u64(const uint8_t *p)
{
    return ((uint64_t)flow_rd_u32(p)) |
           (((uint64_t)flow_rd_u32(p + 4)) << 32);
}

static int16_t flow_rd_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static float flow_rd_f32(const uint8_t *p)
{
    uint32_t u = flow_rd_u32(p);
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

/* ── CRC-8 ───────────────────────────────────────────────────────── */

/* 多项式 0x07, 初值 0x00, 无反射, 无终值异或 (与子板实现逐字节一致) */
uint8_t Flow_Crc8(const uint8_t *d, uint16_t n)
{
    uint8_t c = 0U;
    while (n-- != 0U) {
        uint8_t i;
        c ^= *d++;
        for (i = 0U; i < 8U; i++) {
            c = ((c & 0x80U) != 0U) ? (uint8_t)((c << 1) ^ 0x07U)
                                    : (uint8_t)(c << 1);
        }
    }
    return c;
}

/* ── 航位推算 ────────────────────────────────────────────────────── */

static float flow_clamp_pos(float v)
{
    if (v > FLOW_POS_LIMIT_M) {
        return FLOW_POS_LIMIT_M;
    }
    if (v < -FLOW_POS_LIMIT_M) {
        return -FLOW_POS_LIMIT_M;
    }
    return v;
}

/* 位移(m) = 角度(rad) × 距离(m)
 * 只在 valid==1 且距离为合法正值时累加:
 *   - distance < 0 : 激光未知 -> 冻结
 *   - distance == 0: 0 是合法近距离值, 但乘出来位移恒 0, 冻结等价
 * 单帧 NaN/Inf 或异常大位移一律丢弃, 防止 float 污染累积量 */
static void flow_integrate(const flow_data_t *f)
{
    float dx, dy;

    if (f->valid != 1u) {
        return;
    }
    if (!(f->distance_m > 0.0f)) {
        return;
    }
    if (!isfinite(f->flow_x_rad) || !isfinite(f->flow_y_rad) ||
        !isfinite(f->distance_m)) {
        return;
    }

    dx = f->flow_x_rad * f->distance_m;
    dy = f->flow_y_rad * f->distance_m;

    if (!isfinite(dx) || !isfinite(dy)) {
        return;
    }
    /* 50Hz 下 1 帧位移不可能超过 1m (即 50m/s), 超了就是坏数据 */
    if (fabsf(dx) > 1.0f || fabsf(dy) > 1.0f) {
        return;
    }

    s_posX = flow_clamp_pos(s_posX + FLOW_DISP_X_SIGN * dx);
    s_posY = flow_clamp_pos(s_posY + FLOW_DISP_Y_SIGN * dy);
}

/* ── 帧处理 ──────────────────────────────────────────────────────── */

static void flow_handle_frame(void)
{
    const uint8_t *b = s_rxBuf;
    flow_data_t *f = &s_flow;
    uint8_t seq;

    /* len 必须为 33; 否则视为伪同步头, 丢弃 */
    if (b[2] != FLOW_PAYLOAD_LEN) {
        f->rx_crc_err++;
        return;
    }

    /* CRC 覆盖偏移 2..34, 与 [35] 比较 */
    if (Flow_Crc8(&b[2], FLOW_PAYLOAD_LEN) != b[FLOW_CRC_OFFSET]) {
        f->rx_crc_err++;
        return;
    }

    /* 用 seq 差值估算丢帧 (uint8 回绕自然成立) */
    seq = b[3];
    if (s_haveSeq) {
        uint8_t expect = (uint8_t)(s_lastSeq + 1u);
        if (seq != expect) {
            f->rx_lost += (uint8_t)(seq - expect);
        }
    }
    s_lastSeq = seq;
    s_haveSeq = 1u;

    f->seq            = seq;
    f->time_us        = flow_rd_u64(&b[4]);
    f->integration_us = flow_rd_u32(&b[12]);
    f->flow_x_rad     = flow_rd_f32(&b[16]);
    f->flow_y_rad     = flow_rd_f32(&b[20]);
    f->distance_m     = flow_rd_f32(&b[24]);
    f->quality        = b[28];
    f->status         = b[29];
    f->valid          = b[30];
    f->raw_dx         = flow_rd_i16(&b[31]);
    f->raw_dy         = flow_rd_i16(&b[33]);

    f->rx_ok++;
    f->last_rx_tick = systemTick;
    f->link_ok      = 1u;

    flow_integrate(f);
}

/* ── 公开接口 ────────────────────────────────────────────────────── */

void Flow_Init(void)
{
    memset(&s_flow, 0, sizeof(s_flow));
    memset(s_rxBuf, 0, sizeof(s_rxBuf));
    s_rxIdx   = 0u;
    s_rxState = FLOW_RX_SYNC1;
    s_haveSeq = 0u;
    s_lastSeq = 0u;
    s_posX    = 0.0f;
    s_posY    = 0.0f;
}

void Flow_FeedByte(uint8_t byte)
{
    switch (s_rxState) {
    case FLOW_RX_SYNC1:
        if (byte == FLOW_SYNC1) {
            s_rxState = FLOW_RX_SYNC2;
        }
        break;

    case FLOW_RX_SYNC2:
        if (byte == FLOW_SYNC2) {
            s_rxBuf[0] = FLOW_SYNC1;
            s_rxBuf[1] = FLOW_SYNC2;
            s_rxIdx   = 2u;
            s_rxState = FLOW_RX_PAYLOAD;
        } else if (byte != FLOW_SYNC1) {
            /* 不是 0x55: 重新搜索。若还是 0xAA,
             * 保持 SYNC2 状态以处理 AA AA 55 的重叠同步头 */
            s_rxState = FLOW_RX_SYNC1;
        }
        break;

    default: /* FLOW_RX_PAYLOAD */
        s_rxBuf[s_rxIdx++] = byte;
        if (s_rxIdx >= FLOW_FRAME_SIZE) {
            /* 先回到搜索态, 即使本帧校验失败也能立刻从后续字节重新对齐 */
            s_rxState = FLOW_RX_SYNC1;
            s_rxIdx   = 0u;
            flow_handle_frame();
        }
        break;
    }
}

void Flow_FeedBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if (data == NULL) {
        return;
    }
    for (i = 0u; i < len; i++) {
        Flow_FeedByte(data[i]);
    }
}

void Flow_Update(void)
{
    flow_data_t *f = &s_flow;

    if (f->link_ok && (uint32_t)(systemTick - f->last_rx_tick) > FLOW_TIMEOUT_MS) {
        f->link_ok = 0u;
        f->valid   = 0u;      /* 清除有效标志, 避免窗口拿旧数据积分 */
        s_haveSeq  = 0u;      /* 重连后不要统计一大段假丢帧 */
    }
}

const flow_data_t *Flow_Get(void)
{
    return &s_flow;
}

uint8_t Flow_LinkOk(void)
{
    return s_flow.link_ok;
}

void Flow_ResetPosition(void)
{
    s_posX = 0.0f;
    s_posY = 0.0f;
}

float Flow_GetPosX(void)
{
    return s_posX;
}

float Flow_GetPosY(void)
{
    return s_posY;
}
