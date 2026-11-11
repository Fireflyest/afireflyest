#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>

/* ── NMEA 0183 GPS/BDS 接收 (USART6: TX PA11 / RX PA12) ──────────────
 *
 * 支持语句 (只取显示/控制需要的字段):
 *   $--RMC  时间/日期/经纬度/速度(节)/航向
 *   $--GGA  定位质量/卫星数/HDOP/海拔/大地水准面差距
 *   $--GSA  定位模式 1=无 2=2D 3=3D
 *   $--VTG  航向/地速 (备选速度来源)
 *   $--GLL  经纬度 (备用)
 *   $--ZDA  日期
 *   $--TXT  模块文本 (如 ANTENNA OPEN)
 *   $--GSV  仅计数 (卫星可见性, 不解析)
 *
 * 接收路径: USART6 RXNE 中断 -> 环形缓冲 -> GPS_Update() 排空并逐字节
 * 组句 -> '$'..'\r\n' 切句 -> '*' 后 2 位十六进制校验 (XOR)。
 * 波特率: 38400 8N1。改波特率只改 GPS_BAUDRATE_DEFAULT 一处。
 */

#define GPS_BAUDRATE_DEFAULT    38400u
#define GPS_RING_SIZE           512u    /* 2 的幂: 索引用掩码 */
#define GPS_NMEA_MAX            100u    /* 单句最大长度 (GSV/GSA 约 80) */
#define GPS_MAX_FIELDS          20u     /* 逗号分隔字段上限 */
#define GPS_TIMEOUT_MS          2500u   /* 2.5s 无有效语句 -> 链路断开 */
#define GPS_TXT_MAX             20u
/* 本地时间显示用的时区偏移 (小时) 与标签。NMEA 只给 UTC,
 * 这里只用于窗口对照显示, 不影响解析出的 UTC 字段。
 * 中国大陆 = +8 (CST)。 */
#define GPS_LOCAL_TZ_HOURS      8
#define GPS_LOCAL_TZ_LABEL      "CST"

typedef struct {
    /* ── 位置 (微度, 1e-6 度; 整数避免浮点精度损失) ── */
    int32_t  lat_udeg;      /* 北纬为正, 南纬为负 */
    int32_t  lon_udeg;      /* 东经为正, 西经为负 */
    uint8_t  pos_valid;     /* 1 = 最近一次定位 RMC/GLL 状态为 'A' */

    /* ── 定位质量 ── */
    uint8_t  fix_quality;   /* GGA 字段 6: 0=无效 1=GPS 2=DGPS 4=RTK固定 5=RTK浮动 */
    uint8_t  fix_mode;      /* GSA 字段 2: 1=无 2=2D 3=3D */
    uint8_t  sats_used;     /* GGA 字段 7: 参与定位的卫星数 */
    uint8_t  sats_in_view;  /* GSV 字段 3: 视野内可见卫星数 (0 = 一颗都看不到) */
    uint8_t  sats_in_view_max; /* 上电/清零以来可见卫星数的峰值 (去户外看这个) */
    float    hdop;          /* GGA 字段 8 */

    /* ── 高度 ── */
    float    alt_m;         /* 海拔 (MSL), GGA 字段 9 */
    float    geoid_m;       /* 大地水准面差距, GGA 字段 11 */
    uint8_t  has_alt;

    /* ── 运动 ── */
    float    speed_mps;     /* 地速, RMC 节 × 0.514444 */
    uint8_t  has_speed;
    float    course_deg;    /* 航迹向 (真北, °) */
    uint8_t  has_course;

    /* ── 时间 (UTC) ── */
    uint8_t  hour, minute, second;
    uint8_t  day, month;
    uint16_t year;
    uint8_t  has_time, has_date;

    /* ── 模块文本 (最后一条 $--TXT, 如 "ANTENNA OPEN") ── */
    char     txt[GPS_TXT_MAX];
    uint8_t  has_txt;

    /* ── 最近一条 RMC / GGA 的原始内容 (不含 '$' 和 "*hh")
     * 用途: 窗口里直接看句子字段是不是空的, 例如
     *   "BDRMC,023656.00,A,2240..." -> 字段齐全
     *   "BDRMC,,,,,,,,,,,,,V"       -> 字段全空 */
    char     last_rmc[GPS_NMEA_MAX];
    uint8_t  has_last_rmc;
    char     last_gga[GPS_NMEA_MAX];
    uint8_t  has_last_gga;

    /* ── 最后 8 个原始字节 (窗口 hex 显示, 直接看收到的到底是什么) ── */
    uint8_t  raw[8];

    /* ── 统计/诊断 ── */
    uint32_t rx_raw;        /* ★线上收到的全部字节数 (含 FE/NE/PE 的坏字节) */
    uint32_t rx_raw_bps;    /* ★全部字节的最近 1s 速率 = 线上真实字节率 */
    uint32_t rx_bytes;      /* 进入解析器的字节数 (已排除 FE/NE/PE 的坏字节) */
    uint32_t rx_bps;        /* 其中最近 1s 的速率 */
    uint32_t rx_ascii;      /* 其中可打印 ASCII (0x20~0x7E, 含 \r\n) 的个数 */
    uint32_t rx_hi;         /* 其中 bit7 = 1 (0x80~0xFF) 的个数 */
    uint32_t rx_inv;        /* 其中"按位取反后"是可打印 ASCII 的个数 */
    uint32_t rx_dollar;     /* 出现过多少次 '$' (语句起点) */
    uint32_t rx_lines;      /* 收到的完整句子数 */
    uint32_t rx_bad_csum;   /* 校验和错误数 */
    uint32_t rx_line_err;   /* UART 线路错误 (FE/NE/PE, 字节已丢弃) */
    uint32_t rx_break;      /* FE + 数据 0x00: 线路持续为低 (极性反/RS232/断线) */
    uint32_t rx_hw_ovr;     /* 硬件 ORE 溢出次数 (主循环响应不及时) */
    uint32_t rx_overrun;    /* 环形缓冲溢出次数 */
    uint32_t rx_overlong;   /* 超长句子丢弃次数 */

    uint16_t n_rmc, n_gga, n_gsa, n_gsv, n_vtg, n_gll, n_zda, n_txt;

    uint8_t  link_ok;       /* 1 = GPS_TIMEOUT_MS 内收到过校验通过的语句 */
    uint32_t last_rx_tick;
} gps_data_t;

void GPS_Init(uint32_t baudrate);

/* 运行时改波特率 (默认不需调用, GPS_Init 已按 115200 配置):
 * 会清空统计与组句状态, 便于重新开始 */
void GPS_SetBaudrate(uint32_t baudrate);
uint32_t GPS_GetBaudrate(void);

/* 主循环周期调用: 排空环形缓冲 -> 组句解析 -> 超时检测 */
void GPS_Update(void);

const gps_data_t *GPS_Get(void);
uint8_t GPS_LinkOk(void);

/* "NO" / "2D" / "3D" (GSA 模式; 无 GSA 时按 GGA 质量退化判断) */
const char *GPS_FixStr(void);

void GPS_ResetStats(void);

/* ── 回环自检 (短接 PA11(TX) <-> PA12(RX) 后调用) ──
 * 发一条已知 NMEA 句子并从 RX 收回来, 然后按以下结果判断:
 *   回环后能收到同样内容 (B 增加 ~31, $ 增加 1, A% ~100) -> MCU 侧完全正常,
 *                                                          问题在模块/接线/电平
 *   回环后什么都没有 (B 不变)                              -> UART6 引脚复用或
 *                                                          PA11/PA12 被板上其他电路
 *                                                          (USB/ESD) 拉住
 * 会把发送内容存进回环比对缓冲, 窗口可显示对比结果 */
void GPS_LoopbackTest(void);
uint8_t GPS_LoopbackResult(void);   /* 0=未测 1=通过 2=没收到 3=收到但不一致 */

/* 可打印 ASCII 占比 (0~100, 含 \r\n):
 *   接近 100% -> 字节就是 NMEA 文本, 问题在 '$'/校验/语句类型
 *   很低 (<50%) -> 不是 ASCII 文本 */
uint8_t GPS_AsciiRatio(void);

/* 线上全部字节 (含坏字节) 的累计数与每秒速率 */
uint32_t GPS_RawBytes(void);
uint32_t GPS_RawBps(void);

/* 各星座可见星数明细, 例如 "GP8 BD14 GL0" (最多 3 组, 总长 <= outSize-1)
 * 用途: 说明 sats_in_view 是由哪些星座相加而来
 * 返回写入的字符数 (不含结尾 '\0') */
uint8_t GPS_ViewBreakdown(char *out, uint8_t outSize);

/* bit7=1 的字节占比: 说明收到的是高位为主的字节 */
uint8_t GPS_HiBitRatio(void);

/* "取反后是可打印 ASCII" 的字节占比:
 *   接近 100% 而 GPS_AsciiRatio 接近 0  -> 数据是 ASCII 的按位取反 (极性反) */
uint8_t GPS_InvRatio(void);

/* ── 给 USART6_IRQHandler 调用 ── */
void GPS_RxIsr(uint8_t ch);    /* 好字节: 进环形缓冲 */
void GPS_RawIsr(void);         /* 每收到一个字节就调用 (好坏都算), 测真实字节率 */
void GPS_LineErrorIsr(void);   /* FE/NE/PE: 该字节已损坏 */
void GPS_OverrunIsr(void);     /* ORE: 丢了字节, 但当前字节仍是有效的 */
void GPS_BreakIsr(void);       /* FE + 数据 0x00: 整帧被采成低电平 */

#endif /* __GPS_H */
