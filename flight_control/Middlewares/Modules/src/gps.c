#include "gps.h"
#include "fps.h"

#include <string.h>
#include <stdio.h>

/* 节 -> m/s */
#define GPS_KNOT_TO_MPS     0.514444f

/* ── USART6 接收环形缓冲 (ISR 写 head, 主循环读 tail) ───────────────── */
static uint8_t  s_ring[GPS_RING_SIZE];
static __IO uint16_t s_head;
static __IO uint16_t s_tail;
static __IO uint32_t s_overrun;
static __IO uint32_t s_line_err;
static __IO uint32_t s_hw_ovr;
static __IO uint32_t s_break;
static __IO uint32_t s_raw_bytes;   /* 每收到一个字节(含坏字节) 就 +1 */

/* 字节率统计 */
static uint32_t s_bps_tick;
static uint32_t s_bps_bytes;
static uint32_t s_bps_raw;

/* 回环自检 */
static uint8_t  s_lb_state;      /* 0=未测 1=通过 2=没收到 3=不一致 */
static char     s_lb_expect[40];
static uint16_t s_lb_expect_len;
static uint8_t  s_lb_rx[40];
static __IO uint16_t s_lb_rx_len;
static __IO uint8_t  s_lb_capture;

/* ── 组句状态 ─────────────────────────────────────────────────────── */
static char     s_buf[GPS_NMEA_MAX];
static uint16_t s_len;
static uint8_t  s_active;

/* ── 解析结果 ─────────────────────────────────────────────────────── */
static gps_data_t s_gps;
static uint32_t   s_baud = GPS_BAUDRATE_DEFAULT;

/* ── 小工具 ───────────────────────────────────────────────────────── */

static int gps_hexval(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

/* 十进制无符号解析; 成功返回 1 */
static int gps_parse_u32(const char *s, uint32_t *out)
{
    uint32_t v = 0;
    int any = 0;

    if (s == NULL) {
        return 0;
    }
    while (*s >= '0' && *s <= '9') {
        v = v * 10u + (uint32_t)(*s - '0');
        s++;
        any = 1;
    }
    if (!any) {
        return 0;
    }
    *out = v;
    return 1;
}

/* 浮点解析 (不用 atof/strtod: 体积小, 不受 locale 影响) */
static int gps_parse_f32(const char *s, float *out)
{
    float sign = 1.0f;
    float v = 0.0f;
    float scale = 0.1f;
    int any = 0;

    if (s == NULL || *s == '\0') {
        return 0;
    }
    if (*s == '-') {
        sign = -1.0f;
        s++;
    } else if (*s == '+') {
        s++;
    }
    while (*s >= '0' && *s <= '9') {
        v = v * 10.0f + (float)(*s - '0');
        s++;
        any = 1;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            v += (float)(*s - '0') * scale;
            scale *= 0.1f;
            s++;
            any = 1;
        }
    }
    if (!any) {
        return 0;
    }
    *out = sign * v;
    return 1;
}

/* "ddmm.mmmm" / "dddmm.mmmm" -> 微度 (1e-6 度), 整数运算避免精度损失
 * 例: 2240.61563,N -> 22*1e6 + 40.61563/60*1e6 = 22676927 */
static int32_t gps_dm_to_udeg(const char *s, char hemi)
{
    uint32_t v = 0u;
    uint32_t frac = 0u;
    uint32_t fscale = 1u;
    uint32_t deg, minu, total;
    uint8_t ndig = 0u;
    int neg = 0;

    if (s == NULL) {
        return 0;
    }
    while (*s >= '0' && *s <= '9' && ndig < 9u) {
        v = v * 10u + (uint32_t)(*s - '0');
        s++;
        ndig++;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9' && fscale < 1000000u) {
            frac = frac * 10u + (uint32_t)(*s - '0');
            fscale *= 10u;
            s++;
        }
    }
    if (ndig < 4u) {
        return 0;   /* 至少 ddmm */
    }
    while (fscale < 1000000u) {   /* 小数补齐到 6 位 */
        frac *= 10u;
        fscale *= 10u;
    }

    deg  = v / 100u;
    minu = v % 100u;
    if (minu >= 60u) {
        return 0;
    }
    /* total = deg + (minu + frac/1e6)/60, 全整数并四舍五入 */
    total = deg * 1000000u + (minu * 1000000u + frac + 30u) / 60u;

    if (hemi == 'S' || hemi == 'W') {
        neg = 1;
    }
    return neg ? -(int32_t)total : (int32_t)total;
}

/* "hhmmss.ss" -> hour/minute/second */
static void gps_parse_time(const char *s)
{
    if (s == NULL || strlen(s) < 6u) {
        return;
    }
    if (s[0] < '0' || s[0] > '9' || s[1] < '0' || s[1] > '9' ||
        s[2] < '0' || s[2] > '9' || s[3] < '0' || s[3] > '9' ||
        s[4] < '0' || s[4] > '9' || s[5] < '0' || s[5] > '9') {
        return;
    }
    s_gps.hour   = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
    s_gps.minute = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
    s_gps.second = (uint8_t)((s[4] - '0') * 10 + (s[5] - '0'));
    s_gps.has_time = 1u;
}

/* "ddmmyy" -> day/month/year */
static void gps_parse_date(const char *s)
{
    if (s == NULL || strlen(s) < 6u) {
        return;
    }
    if (s[0] < '0' || s[0] > '9' || s[1] < '0' || s[1] > '9' ||
        s[2] < '0' || s[2] > '9' || s[3] < '0' || s[3] > '9' ||
        s[4] < '0' || s[4] > '9' || s[5] < '0' || s[5] > '9') {
        return;
    }
    s_gps.day   = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
    s_gps.month = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
    s_gps.year  = (uint16_t)(2000u + (uint16_t)((s[4] - '0') * 10 + (s[5] - '0')));
    s_gps.has_date = 1u;
}

/* 校验 XOR 并截断 '*hh'; 返回 1 = 通过 (无校验和字段也放行) */
static int gps_strip_checksum(char *s, uint16_t len)
{
    uint16_t i;

    for (i = 0u; i < len; i++) {
        if (s[i] == '*') {
            int hi = (uint16_t)(i + 1u) < len ? gps_hexval(s[i + 1u]) : -1;
            int lo = (uint16_t)(i + 2u) < len ? gps_hexval(s[i + 2u]) : -1;
            uint8_t ck = 0u;
            uint16_t j;

            if (hi < 0 || lo < 0) {
                s[i] = '\0';
                return 0;
            }
            for (j = 0u; j < i; j++) {
                ck = (uint8_t)(ck ^ (uint8_t)s[j]);
            }
            s[i] = '\0';
            return (ck == (uint8_t)((hi << 4) | lo)) ? 1 : 0;
        }
    }
    return 1;
}

/* 原地按 ',' 切分, 返回字段数 */
static uint8_t gps_split(char *s, char **f, uint8_t maxf)
{
    uint8_t n = 0u;

    f[n++] = s;
    while (*s != '\0') {
        if (*s == ',') {
            *s = '\0';
            if (n >= maxf) {
                return n;
            }
            f[n++] = s + 1;
        }
        s++;
    }
    return n;
}

/* 两个坐标都解析成功才更新; 返回 1 = 成功 */
static int gps_set_pos(const char *lat, char ns, const char *lon, char ew)
{
    int32_t la = gps_dm_to_udeg(lat, ns);
    int32_t lo = gps_dm_to_udeg(lon, ew);

    if (la == 0 || lo == 0) {
        return 0;
    }
    s_gps.lat_udeg = la;
    s_gps.lon_udeg = lo;
    return 1;
}

/* ── 按 talker (语句名前 2 字符) 分别记账 ────────────────────────────
 *
 * 多系统模块会对每个星座分别发同类语句:
 *   $GPGSV / $BDGSV / $GLGSV ...   字段 3 只是"该星座"的可见星数
 *   $GPGGA / $BDGGA / $GNGGA ...   质量/卫星数/高度也是各自的
 *
 * 若一律"后到覆盖", 显示会在不同 talker 之间来回跳。所以按 talker 分开存。
 *
 * 注意两个数来自不同观测域, 不存在"左 <= 右"的必然关系:
 *   GGA 字段 7 (used) = 模块解算时实际用了几颗星, 可能是全系统合计
 *   GSV 字段 3 (view) = 各星座各自跟踪到的星数, 本函数求和
 * 同一 talker 还可能分多组发 GSV (例如两条 BDGSV, 各带自己的总数),
 * 因此每个 talker 取其更新周期内的最大值, 避免后到的小组把总数压低。
 */
#define GPS_MAX_TALKERS     8
#define GPS_TALKER_STALE_MS 3000u   /* 该星座 3s 没报 GSV 就不再计入 */

typedef struct {
    char     key[3];        /* "GP" / "BD" / "GN" ... */
    uint8_t  view;          /* 本星座 GSV 字段 3 的周期内最大值 */
    uint32_t view_tick;     /* 最近一次 GSV 的 systemTick */
    uint8_t  gga_seen;
    uint8_t  gga_qual;
    uint8_t  gga_sats;
    float    gga_hdop;
    float    gga_alt;
    float    gga_geoid;
    uint8_t  gga_has_alt;
} gps_talker_t;

static gps_talker_t s_tk[GPS_MAX_TALKERS];
static uint8_t s_tk_count;

static gps_talker_t *gps_talker_get(const char *name)
{
    uint8_t i;

    if (name == NULL || name[0] == '\0' || name[1] == '\0') {
        return NULL;
    }
    for (i = 0u; i < s_tk_count; i++) {
        if (s_tk[i].key[0] == name[0] && s_tk[i].key[1] == name[1]) {
            return &s_tk[i];
        }
    }
    if (s_tk_count >= GPS_MAX_TALKERS) {
        return NULL;
    }
    memset(&s_tk[s_tk_count], 0, sizeof(s_tk[0]));
    s_tk[s_tk_count].key[0] = name[0];
    s_tk[s_tk_count].key[1] = name[1];
    s_tk_count++;
    return &s_tk[s_tk_count - 1u];
}

/* 取该星座当前的可见星数; 超时未更新则清零 (该星座已不再上报) */
static uint8_t gps_talker_view_now(gps_talker_t *tk)
{
    if (tk->view_tick == 0u) {
        return 0u;
    }
    if ((uint32_t)(systemTick - tk->view_tick) > GPS_TALKER_STALE_MS) {
        tk->view = 0u;
        tk->view_tick = 0u;
        return 0u;
    }
    return tk->view;
}

/* 把各 talker 的记录聚合到对外字段 */
static void gps_talker_aggregate(void)
{
    uint16_t sum = 0u;
    int best = -1;
    uint8_t i;

    for (i = 0u; i < s_tk_count; i++) {
        sum = (uint16_t)(sum + gps_talker_view_now(&s_tk[i]));

        if (!s_tk[i].gga_seen) {
            continue;
        }
        if (best < 0 ||
            s_tk[i].gga_qual > s_tk[best].gga_qual ||
            (s_tk[i].gga_qual == s_tk[best].gga_qual &&
             s_tk[i].gga_sats > s_tk[best].gga_sats)) {
            best = (int)i;
        }
    }

    if (sum > 255u) {
        sum = 255u;
    }
    s_gps.sats_in_view = (uint8_t)sum;
    if (s_gps.sats_in_view > s_gps.sats_in_view_max) {
        s_gps.sats_in_view_max = s_gps.sats_in_view;
    }

    if (best >= 0) {
        s_gps.fix_quality = s_tk[best].gga_qual;
        s_gps.sats_used   = s_tk[best].gga_sats;
        s_gps.hdop        = s_tk[best].gga_hdop;
        s_gps.alt_m       = s_tk[best].gga_alt;
        s_gps.geoid_m     = s_tk[best].gga_geoid;
        s_gps.has_alt     = s_tk[best].gga_has_alt;
    }
}

/* ── 各语句处理 ───────────────────────────────────────────────────── */

static void gps_do_rmc(char **f, uint8_t nf)
{
    float v;

    s_gps.n_rmc++;
    if (nf < 10u) {
        return;
    }
    gps_parse_time(f[1]);
    gps_parse_date(f[9]);

    if (f[2][0] == 'A' && f[3][0] != '\0' && f[5][0] != '\0') {
        s_gps.pos_valid = (uint8_t)gps_set_pos(f[3], f[4][0], f[5], f[6][0]);
    } else {
        s_gps.pos_valid = 0u;
    }

    if (gps_parse_f32(f[7], &v)) {
        s_gps.speed_mps = v * GPS_KNOT_TO_MPS;
        s_gps.has_speed = 1u;
    }
    if (gps_parse_f32(f[8], &v)) {
        s_gps.course_deg = v;
        s_gps.has_course = 1u;
    }
}

static void gps_do_gga(char **f, uint8_t nf)
{
    gps_talker_t *tk;
    uint32_t u;
    float v;

    s_gps.n_gga++;
    if (nf < 10u) {
        return;
    }
    gps_parse_time(f[1]);

    if (f[2][0] != '\0' && f[4][0] != '\0') {
        (void)gps_set_pos(f[2], f[3][0], f[4], f[5][0]);
    }

    /* 质量/卫星数/高度按 talker 分别记录, 由 aggregate 选出"最好"的一条,
     * 避免某个星座的 GGA (可能是 0 颗) 覆盖掉有效解 */
    tk = gps_talker_get(f[0]);
    if (tk == NULL) {
        return;
    }
    tk->gga_seen = 1u;
    tk->gga_qual = gps_parse_u32(f[6], &u) ? (uint8_t)u : 0u;
    tk->gga_sats = gps_parse_u32(f[7], &u) ? (uint8_t)u : 0u;
    if (gps_parse_f32(f[8], &v)) {
        tk->gga_hdop = v;
    }
    if (gps_parse_f32(f[9], &v)) {
        tk->gga_alt = v;
        tk->gga_has_alt = 1u;
    }
    if (nf > 11u && gps_parse_f32(f[11], &v)) {
        tk->gga_geoid = v;
    }

    gps_talker_aggregate();
}

/* GSA 只有固定几个字段可靠 (字段数随参与卫星数变化, DOP 位置会移位),
 * 所以只取 [1]=模式 和 [2]=定位类型, DOP 用 GGA 的 HDOP */
static void gps_do_gsa(char **f, uint8_t nf)
{
    uint32_t u;

    s_gps.n_gsa++;
    if (nf < 3u) {
        return;
    }
    if (gps_parse_u32(f[2], &u)) {
        s_gps.fix_mode = (uint8_t)u;
    }
}

static void gps_do_vtg(char **f, uint8_t nf)
{
    float v;

    s_gps.n_vtg++;
    /* 优先 "x.x,N" (节), 其次 "x.x,K" (km/h) */
    if (nf > 6u && f[6][0] == 'N' && gps_parse_f32(f[5], &v)) {
        s_gps.speed_mps = v * GPS_KNOT_TO_MPS;
        s_gps.has_speed = 1u;
    } else if (nf > 8u && f[8][0] == 'K' && gps_parse_f32(f[7], &v)) {
        s_gps.speed_mps = v / 3.6f;
        s_gps.has_speed = 1u;
    }
    if (nf > 1u && gps_parse_f32(f[1], &v)) {
        s_gps.course_deg = v;
        s_gps.has_course = 1u;
    }
}

static void gps_do_gll(char **f, uint8_t nf)
{
    s_gps.n_gll++;
    if (nf < 7u) {
        return;
    }
    gps_parse_time(f[5]);
    if (f[6][0] == 'A' && f[1][0] != '\0' && f[3][0] != '\0') {
        s_gps.pos_valid = (uint8_t)gps_set_pos(f[1], f[2][0], f[3], f[4][0]);
    }
}

static void gps_do_zda(char **f, uint8_t nf)
{
    uint32_t u;

    s_gps.n_zda++;
    if (nf < 5u) {
        return;
    }
    gps_parse_time(f[1]);
    if (gps_parse_u32(f[2], &u) && u >= 1u && u <= 31u) {
        s_gps.day = (uint8_t)u;
    }
    if (gps_parse_u32(f[3], &u) && u >= 1u && u <= 12u) {
        s_gps.month = (uint8_t)u;
    }
    if (gps_parse_u32(f[4], &u) && u >= 2000u && u <= 2099u) {
        s_gps.year = (uint16_t)u;
    }
    s_gps.has_date = 1u;
}

static void gps_do_txt(char **f, uint8_t nf)
{
    s_gps.n_txt++;
    /* 文本恒为最后一个字段:
     *   4 字段式 $GPTXT,01,01,ANTENNA OPEN  (u-blox 等)
     *   5 字段式 $GPTXT,01,01,01,ANTENNA OPEN (Beitian 等) */
    if (nf < 2u) {
        return;
    }
    strncpy(s_gps.txt, f[nf - 1u], GPS_TXT_MAX - 1u);
    s_gps.txt[GPS_TXT_MAX - 1u] = '\0';
    s_gps.has_txt = (s_gps.txt[0] != '\0') ? 1u : 0u;
}

/* GSV 字段:
 *   [1] 本次报文总数  [2] 本报文序号  [3] 视野内卫星总数(该星座)
 *   之后每 4 个字段一组: PRN / 仰角 / 方位 / SNR
 * 只取 [3], 按 talker 存入各自记录 (取周期内最大值),
 * 总和由 gps_talker_aggregate 计算 */
static void gps_do_gsv(char **f, uint8_t nf)
{
    gps_talker_t *tk;
    uint32_t u;

    s_gps.n_gsv++;
    if (nf < 4u) {
        return;
    }
    if (!gps_parse_u32(f[3], &u) || u > 255u) {
        return;
    }
    tk = gps_talker_get(f[0]);
    if (tk == NULL) {
        return;
    }
    /* 同一 talker 可能分多组发 GSV (每组自带总数), 取最大值避免低估 */
    if ((uint8_t)u > tk->view || tk->view_tick == 0u) {
        tk->view = (uint8_t)u;
    }
    tk->view_tick = systemTick;
    gps_talker_aggregate();
}

/* ── 单句解析 ─────────────────────────────────────────────────────── */

static void gps_parse_line(char *s, uint16_t len)
{
    char *f[GPS_MAX_FIELDS];
    char keep[GPS_NMEA_MAX];
    uint8_t nf;
    const char *type;
    size_t namelen;

    s_gps.rx_lines++;

    if (!gps_strip_checksum(s, len)) {
        s_gps.rx_bad_csum++;
        return;
    }

    /* 校验通过才认为链路正常 (排除波特率不匹配的乱码) */
    s_gps.link_ok = 1u;
    s_gps.last_rx_tick = systemTick;

    /* 备份原文: 下面 gps_split 会把 ',' 就地改成 '\0', 之后再也拼不回原句。
     * 只保留 RMC/GGA (带位置的句子) 供窗口回显。 */
    strncpy(keep, s, sizeof(keep) - 1u);
    keep[sizeof(keep) - 1u] = '\0';

    nf = gps_split(s, f, GPS_MAX_FIELDS);
    if (nf < 1u) {
        return;
    }
    namelen = strlen(f[0]);
    if (namelen < 3u) {
        return;
    }
    /* 忽略 talker (GP/BD/GN/...), 只看后 3 字符 */
    type = f[0] + (namelen - 3u);

    if (strcmp(type, "RMC") == 0) {
        memcpy(s_gps.last_rmc, keep, sizeof(s_gps.last_rmc));
        s_gps.has_last_rmc = 1u;
        gps_do_rmc(f, nf);
    } else if (strcmp(type, "GGA") == 0) {
        memcpy(s_gps.last_gga, keep, sizeof(s_gps.last_gga));
        s_gps.has_last_gga = 1u;
        gps_do_gga(f, nf);
    } else if (strcmp(type, "GSA") == 0) {
        gps_do_gsa(f, nf);
    } else if (strcmp(type, "VTG") == 0) {
        gps_do_vtg(f, nf);
    } else if (strcmp(type, "GLL") == 0) {
        gps_do_gll(f, nf);
    } else if (strcmp(type, "ZDA") == 0) {
        gps_do_zda(f, nf);
    } else if (strcmp(type, "TXT") == 0) {
        gps_do_txt(f, nf);
    } else if (strcmp(type, "GSV") == 0) {
        gps_do_gsv(f, nf);
    }
}

/* ── 字节流组句 ───────────────────────────────────────────────────── */

static void gps_feed(uint8_t ch)
{
    s_gps.rx_bytes++;

    /* ── 三个统计量, 用来客观描述"线上传的到底是什么" ──
     * A%: 字节本身就是可打印 ASCII (NMEA 应 ≈100)
     * H%: bit7 = 1 的占比
     * I%: "取反后"是可打印 ASCII 的占比 (≈100 且 A%≈0 => 数据被按位取反) */
    if ((ch >= 0x20u && ch <= 0x7Eu) || ch == '\r' || ch == '\n') {
        s_gps.rx_ascii++;
    }
    if ((ch & 0x80u) != 0u) {
        s_gps.rx_hi++;
    }
    {
        uint8_t inv = (uint8_t)(~ch);
        if ((inv >= 0x20u && inv <= 0x7Eu) || inv == '\r' || inv == '\n') {
            s_gps.rx_inv++;
        }
    }

    /* 滚动保存最后 8 字节: 窗口里直接看原始数据 */
    memmove(&s_gps.raw[0], &s_gps.raw[1], sizeof(s_gps.raw) - 1u);
    s_gps.raw[sizeof(s_gps.raw) - 1u] = ch;

    if (ch == '$') {
        s_gps.rx_dollar++;
        s_len = 0u;
        s_active = 1u;
        return;
    }
    if (!s_active) {
        return;
    }
    if (ch == '\r' || ch == '\n') {
        if (s_len >= 6u) {
            s_buf[s_len] = '\0';
            gps_parse_line(s_buf, s_len);
        }
        s_len = 0u;
        s_active = 0u;
        return;
    }
    if (s_len < (GPS_NMEA_MAX - 1u)) {
        s_buf[s_len++] = (char)ch;
    } else {
        /* 超长: 丢弃, 等下一个 '$' 重新对齐 */
        s_len = 0u;
        s_active = 0u;
        s_gps.rx_overlong++;
    }
}

/* ── 公开接口 ─────────────────────────────────────────────────────── */

void GPS_Init(uint32_t baudrate)
{
    NVIC_InitTypeDef n;

    memset(&s_gps, 0, sizeof(s_gps));
    memset(s_ring, 0, sizeof(s_ring));
    s_head = 0u;
    s_tail = 0u;
    s_len = 0u;
    s_active = 0u;
    s_overrun = 0u;
    s_line_err = 0u;
    s_hw_ovr = 0u;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    n.NVIC_IRQChannel = USART6_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 12;
    n.NVIC_IRQChannelSubPriority = 1;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);

    GPS_SetBaudrate(baudrate);
    /* 不用 IDLE 辅助切句: NMEA 有空闲间隔, 且 '\r\n' 已足够可靠 */
}

void GPS_SetBaudrate(uint32_t baudrate)
{
    USART_InitTypeDef u;

    if (baudrate == 0u) {
        baudrate = GPS_BAUDRATE_DEFAULT;
    }
    s_baud = baudrate;

    USART_Cmd(USART6, DISABLE);
    u.USART_BaudRate = baudrate;
    u.USART_WordLength = USART_WordLength_8b;
    u.USART_StopBits = USART_StopBits_1;
    u.USART_Parity = USART_Parity_No;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    u.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &u);
    USART_Cmd(USART6, ENABLE);
    /* USART_Init 只改 CR1 的 M/PCE/PS/TE/RE, RXNEIE 会保留;
     * 这里再显式打开一次, 不依赖驱动实现细节 */
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    /* 重新开始: 丢弃环形缓冲里的旧波特率数据, 清统计与链路标志 */
    s_head = 0u;
    s_tail = 0u;
    s_len = 0u;
    s_active = 0u;
    s_overrun = 0u;
    s_line_err = 0u;
    s_hw_ovr = 0u;
    GPS_ResetStats();
    s_gps.link_ok = 0u;
    s_gps.pos_valid = 0u;
    s_gps.fix_mode = 0u;
    s_gps.fix_quality = 0u;
    s_gps.sats_used = 0u;
    s_gps.has_speed = 0u;
    s_gps.has_course = 0u;
}

uint32_t GPS_GetBaudrate(void)
{
    return s_baud;
}

void GPS_Update(void)
{
    /* 排空环形缓冲: 每次调用把已到字节全部解析完, 不阻塞主循环 */
    while (s_tail != s_head) {
        uint8_t ch = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) & (GPS_RING_SIZE - 1u));
        gps_feed(ch);
    }

    s_gps.rx_overrun  = s_overrun;
    s_gps.rx_line_err = s_line_err;
    s_gps.rx_hw_ovr   = s_hw_ovr;
    s_gps.rx_break    = s_break;

    /* 字节率 (1s 窗口): 直接区分"模块真实 NMEA"与"空脚/电平错误的噪声":
     *   ~1100 B/s = 模块实际数据率 (PS 差一点的模块也可能 900~1300)
     *   ~11500 B/s = 线在乱跳 (空脚/悬空/串扰), 不是有效 UART
     *   ~0 B/s     = 没有数据 */
    if ((uint32_t)(systemTick - s_bps_tick) >= 1000u) {
        uint32_t dt = (uint32_t)(systemTick - s_bps_tick);
        uint32_t db = s_gps.rx_bytes - s_bps_bytes;
        uint32_t dr = s_raw_bytes - s_bps_raw;
        s_gps.rx_bps = (dt != 0u) ? (uint32_t)(((uint64_t)db * 1000u) / dt) : 0u;
        s_gps.rx_raw_bps = (dt != 0u) ? (uint32_t)(((uint64_t)dr * 1000u) / dt) : 0u;
        s_bps_tick = systemTick;
        s_bps_bytes = s_gps.rx_bytes;
        s_bps_raw = s_raw_bytes;
    }

    s_gps.rx_raw = s_raw_bytes;

    if (s_gps.link_ok &&
        (uint32_t)(systemTick - s_gps.last_rx_tick) > GPS_TIMEOUT_MS) {
        s_gps.link_ok = 0u;
        s_gps.pos_valid = 0u;
        s_gps.fix_mode = 0u;
        s_gps.fix_quality = 0u;
        s_gps.sats_used = 0u;
        s_gps.has_course = 0u;
        s_gps.has_speed = 0u;
    }
}

const gps_data_t *GPS_Get(void)
{
    return &s_gps;
}

uint8_t GPS_LinkOk(void)
{
    return s_gps.link_ok;
}

const char *GPS_FixStr(void)
{
    if (!s_gps.link_ok) {
        return "NO";
    }
    if (s_gps.fix_mode == 3u) {
        return "3D";
    }
    if (s_gps.fix_mode == 2u) {
        return "2D";
    }
    /* 没有 GSA: 用 GGA 定位质量退化判断 */
    if (s_gps.fix_quality >= 1u) {
        return "3D";
    }
    return "NO";
}

void GPS_ResetStats(void)
{
    s_gps.rx_bytes = 0u;
    s_gps.rx_ascii = 0u;
    s_gps.rx_hi = 0u;
    s_gps.rx_inv = 0u;
    s_gps.rx_dollar = 0u;
    s_gps.rx_lines = 0u;
    s_gps.rx_bad_csum = 0u;
    s_gps.rx_line_err = 0u;
    s_gps.rx_overrun = 0u;
    s_gps.rx_overlong = 0u;
    s_gps.n_rmc = 0u;
    s_gps.n_gga = 0u;
    s_gps.n_gsa = 0u;
    s_gps.n_gsv = 0u;
    s_gps.n_vtg = 0u;
    s_gps.n_gll = 0u;
    s_gps.n_zda = 0u;
    s_gps.n_txt = 0u;
    s_overrun = 0u;
    s_line_err = 0u;
    s_hw_ovr = 0u;
    s_break = 0u;
    s_bps_tick = systemTick;
    s_bps_bytes = s_gps.rx_bytes;
    s_bps_raw = s_raw_bytes;
    s_gps.rx_bps = 0u;
    s_gps.rx_raw_bps = 0u;

    /* 各星座记账也清零 */
    memset(s_tk, 0, sizeof(s_tk));
    s_tk_count = 0u;
    s_gps.sats_in_view = 0u;
    s_gps.sats_in_view_max = 0u;
}

void GPS_RxIsr(uint8_t ch)
{
    uint16_t next = (uint16_t)((s_head + 1u) & (GPS_RING_SIZE - 1u));

    /* 回环自检期间同步抓一份原始字节 (在 ISR 里做, 避免主循环阻塞影响) */
    if (s_lb_capture && s_lb_rx_len < sizeof(s_lb_rx)) {
        s_lb_rx[s_lb_rx_len++] = ch;
    }

    if (next != s_tail) {
        s_ring[s_head] = ch;
        s_head = next;
    } else {
        s_overrun++;    /* 满: 丢弃, 计数供诊断 */
    }
}

/* 每收到一个字节调用一次 (好坏都算), 用于测线上真实字节率 */
void GPS_RawIsr(void)
{
    s_raw_bytes++;
}

void GPS_LineErrorIsr(void)
{
    s_line_err++;
}

void GPS_OverrunIsr(void)
{
    s_hw_ovr++;
}

void GPS_BreakIsr(void)
{
    s_break++;
}

/* ── 回环自检 ─────────────────────────────────────────────────────
 * ★ 做之前必须先断开模块的 TX 与 PA12 的连接, 再短接 PA11(TX) <-> PA12(RX),
 *   否则模块的 TX 与 MCU 的 TX 直接对接, 会互相灌电流。
 * 发一条固定句子再从 RX 收回来比对。
 * 发送用轮询 TXE/TC (约 3ms 阻塞, 只在人工按键时调用)。
 * 收发完清空环形缓冲与统计, 不影响正常解析。 */
void GPS_LoopbackTest(void)
{
    /* 内容带时间戳变化, 避免缓存/假回环混淆 */
    uint32_t n = (uint32_t)systemTick;
    char body[32];
    int blen;
    uint8_t ck = 0u;
    int i;

    blen = snprintf(body, sizeof(body), "GPTXT,01,01,02,LB%lu", (unsigned long)(n % 100000u));
    if (blen <= 0) {
        return;
    }

    /* 先停采集, 清统计 + 清环形缓冲 */
    s_lb_capture = 0u;
    s_head = 0u;
    s_tail = 0u;
    s_len = 0u;
    s_active = 0u;
    GPS_ResetStats();

    /* 组装: $body*HH\r\n */
    s_lb_expect[0] = '$';
    memcpy(&s_lb_expect[1], body, (size_t)blen);
    for (i = 0; i < blen; i++) {
        ck ^= (uint8_t)body[i];
    }
    snprintf(&s_lb_expect[1 + blen], sizeof(s_lb_expect) - 1u - (size_t)blen,
             "*%02X\r\n", (unsigned)ck);
    s_lb_expect_len = (uint16_t)strlen(s_lb_expect);
    s_lb_rx_len = 0u;
    s_lb_state = 0u;

    /* 开始采集, 然后发送 */
    s_lb_capture = 1u;
    for (i = 0; i < (int)s_lb_expect_len; i++) {
        uint32_t guard = 0u;
        while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) {
            if (++guard > 1000000u) {
                break;
            }
        }
        USART_SendData(USART6, (uint16_t)(uint8_t)s_lb_expect[i]);
    }
    {
        uint32_t guard = 0u;
        while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET) {
            if (++guard > 1000000u) {
                break;
            }
        }
    }

    /* 等回环字节全部到齐 (31 字节 @115200 ≈ 2.7ms, 留 10ms 余量)。
     * 等待期间 ISR 会持续把字节抓进 s_lb_rx */
    for (i = 0; i < 10; i++) {
        Delay_ms(1);
        if (s_lb_rx_len >= s_lb_expect_len) {
            break;
        }
    }
    s_lb_capture = 0u;

    if (s_lb_rx_len == 0u) {
        s_lb_state = 2u;        /* 什么都没收到 */
    } else if (s_lb_rx_len == s_lb_expect_len &&
               memcmp(s_lb_rx, s_lb_expect, s_lb_expect_len) == 0) {
        s_lb_state = 1u;        /* 完全一致 */
    } else {
        s_lb_state = 3u;        /* 收到但不一致 */
    }

    /* 清掉回环字节在统计里的影响, 回到正常接收状态 */
    s_head = 0u;
    s_tail = 0u;
    s_len = 0u;
    s_active = 0u;
    GPS_ResetStats();
}

uint8_t GPS_LoopbackResult(void)
{
    return s_lb_state;
}

uint8_t GPS_AsciiRatio(void)
{
    uint32_t total = s_gps.rx_bytes;
    uint64_t scaled;

    if (total == 0u) {
        return 0u;
    }
    /* 用 64 位避免 ratio*100 溢出后再除 */
    scaled = ((uint64_t)s_gps.rx_ascii * 100u) / (uint64_t)total;
    if (scaled > 100u) {
        scaled = 100u;
    }
    return (uint8_t)scaled;
}

uint32_t GPS_RawBytes(void)
{
    return s_raw_bytes;
}

uint32_t GPS_RawBps(void)
{
    return s_gps.rx_raw_bps;
}

/* 各星座可见星数明细, 形如 "GP8 BD14 GL0"
 * 只列有 GSV 上报过的星座; 3s 未更新的视为 0 并跳过 */
uint8_t GPS_ViewBreakdown(char *out, uint8_t outSize)
{
    uint8_t n = 0u;
    uint8_t i;

    if (out == NULL || outSize < 4u) {
        return 0u;
    }
    out[0] = '\0';

    for (i = 0u; i < s_tk_count; i++) {
        uint8_t v = gps_talker_view_now(&s_tk[i]);
        int w;

        if (s_tk[i].view_tick == 0u && v == 0u) {
            continue;   /* 该 talker 没有 GSV */
        }
        if ((uint16_t)(n + 8u) >= (uint16_t)outSize) {
            break;
        }
        w = snprintf(&out[n], (size_t)(outSize - n), "%s%c%c%u",
                     (n > 0u) ? " " : "",
                     s_tk[i].key[0], s_tk[i].key[1], (unsigned)v);
        if (w <= 0) {
            break;
        }
        n = (uint8_t)(n + (uint8_t)w);
    }

    if (n == 0u) {
        snprintf(out, outSize, "no GSV");
        n = (uint8_t)strlen(out);
    }
    return n;
}

uint8_t GPS_HiBitRatio(void)
{
    if (s_gps.rx_bytes == 0u) {
        return 0u;
    }
    {
        uint64_t v = ((uint64_t)s_gps.rx_hi * 100u) / (uint64_t)s_gps.rx_bytes;
        return (uint8_t)((v > 100u) ? 100u : v);
    }
}

uint8_t GPS_InvRatio(void)
{
    if (s_gps.rx_bytes == 0u) {
        return 0u;
    }
    {
        uint64_t v = ((uint64_t)s_gps.rx_inv * 100u) / (uint64_t)s_gps.rx_bytes;
        return (uint8_t)((v > 100u) ? 100u : v);
    }
}
