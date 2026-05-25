/**
 * @file mmc5983ma.c
 * @brief MMC5983MA 磁传感器驱动 — 基于 spi_sensor.h 框架
 *        仅依赖 SPI_Sensor_ReadBytes / SPI_Sensor_WriteBytes
 *        数据手册: MEMSIC MMC5983MA Rev. A (2/25/2019)
 */

#include "mmc5983ma.h"
#include <math.h>

/* ═══════════════════════════════════════════════════
 *  内部延时
 * ═══════════════════════════════════════════════════ */

static mmc5983ma_delay_fn_t s_delay = NULL;

void MMC5983MA_SetDelay(mmc5983ma_delay_fn_t fn) {
    s_delay = fn;
}

static void delay_ms(uint32_t ms) {
    if (s_delay)
        s_delay(ms);
}

/* ═══════════════════════════════════════════════════
 *  底层寄存器读写
 *  SPI 格式: bit0=R/W, bit1=X, bit[7:2]=地址, bit[15:8]=数据
 *  每次传输 16-bit 帧，多字节则追加 8-bit 块
 * ═══════════════════════════════════════════════════ */

static uint8_t reg_read(const mmc5983ma_dev_t* dev, uint8_t reg) {
    uint8_t val = 0;
    SPI_Sensor_ReadBytes(dev->sensor_id, reg, &val, 1);
    return val;
}

static void reg_write(const mmc5983ma_dev_t* dev, uint8_t reg, uint8_t val) {
    SPI_Sensor_WriteBytes(dev->sensor_id, reg, &val, 1);
}

static void reg_read_burst(const mmc5983ma_dev_t* dev,
                           uint8_t reg,
                           uint8_t* buf,
                           uint16_t len) {
    SPI_Sensor_ReadBytes(dev->sensor_id, reg, buf, len);
}

static void reg_rmw(const mmc5983ma_dev_t* dev,
                    uint8_t reg,
                    uint8_t mask,
                    uint8_t val) {
    uint8_t tmp = reg_read(dev, reg);
    tmp = (tmp & ~mask) | (val & mask);
    reg_write(dev, reg, tmp);
}

/* ═══════════════════════════════════════════════════
 *  基础
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_WhoAmI(mmc5983ma_dev_t* dev, uint8_t* id) {
    if (!dev || !id)
        return MMC_ERR_PARAM;
    *id = reg_read(dev, MMC_REG_PRODUCT_ID);
    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_SoftReset(mmc5983ma_dev_t* dev) {
    if (!dev)
        return MMC_ERR_PARAM;

    /* SW_RST = 1, 写入 CTRL1 */
    reg_write(dev, MMC_REG_CTRL1, MMC_CTRL1_SW_RST);

    /* 上电时间 10ms */
    delay_ms(15);

    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_Init(mmc5983ma_dev_t* dev) {
    uint8_t pid;
    uint8_t status;

    if (!dev)
        return MMC_ERR_PARAM;

    /* 软复位到已知状态 */
    MMC5983MA_SoftReset(dev);

    /* 验证 Product ID */
    pid = reg_read(dev, MMC_REG_PRODUCT_ID);
    if (pid != MMC_PRODUCT_ID)
        return MMC_ERR_PRODUCT_ID;

    /* 重新读 OTP 并等待完成 */
    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_OTP_READ);
    delay_ms(1);

    {
        uint32_t timeout = 100;
        do {
            status = reg_read(dev, MMC_REG_STATUS);
            if (status & MMC_STATUS_OTP_RD_DONE)
                break;
            delay_ms(1);
        } while (--timeout);
        if (timeout == 0)
            return MMC_ERR_OTP;
    }

    /* 清状态位 */
    reg_write(dev, MMC_REG_STATUS,
              MMC_STATUS_MEAS_M_DONE | MMC_STATUS_MEAS_T_DONE);

    /* 默认 16-bit, BW=00 */
    dev->bw = MMC_BW_00;
    dev->use_18bit = 0;

    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  配置
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_SetBandwidth(mmc5983ma_dev_t* dev,
                                       mmc5983ma_bw_t bw) {
    if (!dev)
        return MMC_ERR_PARAM;

    /* CTRL1: BW[1:0] = bit[1:0] */
    reg_rmw(dev, MMC_REG_CTRL1, 0x03, (uint8_t)bw);

    dev->bw = bw;
    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_EnableIntMeasDone(mmc5983ma_dev_t* dev,
                                            uint8_t enable) {
    if (!dev)
        return MMC_ERR_PARAM;

    reg_rmw(dev, MMC_REG_CTRL0, MMC_CTRL0_INT_MEAS_EN,
            enable ? MMC_CTRL0_INT_MEAS_EN : 0);
    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_EnableAutoSR(mmc5983ma_dev_t* dev,
                                       uint8_t enable) {
    if (!dev)
        return MMC_ERR_PARAM;

    reg_rmw(dev, MMC_REG_CTRL0, MMC_CTRL0_AUTO_SR_EN,
            enable ? MMC_CTRL0_AUTO_SR_EN : 0);
    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  连续测量模式
 *  CTRL2 (0x0B):
 *    bit [2:0] = CM_Freq[2:0]
 *    bit [3]   = Cmm_en
 *    bit [6:4] = Prd_set[2:0]
 *    bit [7]   = En_prd_set
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_EnableContinuousMode(mmc5983ma_dev_t* dev,
                                               mmc5983ma_cm_freq_t freq,
                                               mmc5983ma_prd_set_t prd_set) {
    uint8_t val;
    if (!dev)
        return MMC_ERR_PARAM;
    if (freq == MMC_CM_FREQ_OFF)
        return MMC_ERR_PARAM;

    /* 需要同时使能 Auto_SR_en */
    MMC5983MA_EnableAutoSR(dev, 1);

    val = (uint8_t)(MMC_CTRL2_CMM_EN                   /* Cmm_en=1 */
                    | (freq & 0x07)                    /* CM_Freq */
                    | (((uint8_t)prd_set & 0x07) << 4) /* Prd_set */
                    | MMC_CTRL2_EN_PRD_SET);           /* En_prd_set */

    reg_write(dev, MMC_REG_CTRL2, val);

    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_DisableContinuousMode(mmc5983ma_dev_t* dev) {
    if (!dev)
        return MMC_ERR_PARAM;

    reg_write(dev, MMC_REG_CTRL2, 0x00);
    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  SET / RESET
 *  CTRL0 (0x09):
 *    bit 3 = Set   (写1触发，自清除)
 *    bit 4 = Reset (写1触发，自清除)
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_Set(mmc5983ma_dev_t* dev) {
    if (!dev)
        return MMC_ERR_PARAM;

    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_SET);
    /* SET 脉冲 500ns，自清除，短暂等待 */
    delay_ms(1);
    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_Reset(mmc5983ma_dev_t* dev) {
    if (!dev)
        return MMC_ERR_PARAM;

    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_RESET);
    delay_ms(1);
    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  等待测量完成
 * ═══════════════════════════════════════════════════ */

/** 根据 BW 估算最大测量等待时间(ms) */
static const uint8_t meas_time_ms[4] = {
    10, /* BW=00: 8ms */
    6,  /* BW=01: 4ms */
    4,  /* BW=10: 2ms */
    2,  /* BW=11: 0.5ms */
};

static mmc5983ma_err_t wait_mag_done(mmc5983ma_dev_t* dev) {
    uint32_t timeout = (uint32_t)meas_time_ms[dev->bw & 0x03] + 10;
    uint32_t t = 0;
    uint8_t status;

    do {
        status = reg_read(dev, MMC_REG_STATUS);
        if (status & MMC_STATUS_MEAS_M_DONE)
            return MMC_OK;
        delay_ms(1);
    } while (++t < timeout);

    return MMC_ERR_TIMEOUT;
}

static mmc5983ma_err_t wait_temp_done(mmc5983ma_dev_t* dev) {
    uint32_t timeout = 20;
    uint32_t t = 0;
    uint8_t status;

    do {
        status = reg_read(dev, MMC_REG_STATUS);
        if (status & MMC_STATUS_MEAS_T_DONE)
            return MMC_OK;
        delay_ms(1);
    } while (++t < timeout);

    return MMC_ERR_TIMEOUT;
}

/* ═══════════════════════════════════════════════════
 *  读取 XYZ 原始数据
 *
 *  寄存器布局 (从 0x00 连续读 7 字节):
 *    [0] Xout0: Xout[17:10]
 *    [1] Xout1: Xout[9:2]
 *    [2] Yout0: Yout[17:10]
 *    [3] Yout1: Yout[9:2]
 *    [4] Zout0: Zout[17:10]
 *    [5] Zout1: Zout[9:2]
 *    [6] XYZout2: Xout[1:0] Yout[1:0] Zout[1:0]
 *
 *  Burst read 支持地址自增，一次读 7 字节即可。
 * ═══════════════════════════════════════════════════ */

static void parse_xyz_raw(const uint8_t buf[7],
                          mmc5983ma_raw_data_t* raw,
                          uint8_t use_18bit) {
    uint32_t x, y, z;

    /* 高 8 位 + 中 8 位 */
    x = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2);
    y = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2);
    z = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2);

    /* XYZout2 (buf[6]): X[1:0] Y[1:0] Z[1:0] */
    x |= ((uint32_t)buf[6] >> 6) & 0x03;
    y |= ((uint32_t)buf[6] >> 4) & 0x03;
    z |= ((uint32_t)buf[6] >> 2) & 0x03;

    if (!use_18bit) {
        /* 16-bit 模式: 右移 2 位，忽略最低 2 位 */
        x >>= 2;
        y >>= 2;
        z >>= 2;
    }

    raw->x = x;
    raw->y = y;
    raw->z = z;
}

/* ═══════════════════════════════════════════════════
 *  单次测量（触发 + 等待 + 读取）
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_ReadMagRaw(mmc5983ma_dev_t* dev,
                                     mmc5983ma_raw_data_t* raw) {
    mmc5983ma_err_t r;
    uint8_t buf[7];

    if (!dev || !raw)
        return MMC_ERR_PARAM;

    /* 触发磁场测量 */
    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_TM_M);

    /* 等待完成 */
    r = wait_mag_done(dev);
    if (r)
        return r;

    /* burst read: Xout0(0x00) ~ XYZout2(0x06) */
    reg_read_burst(dev, MMC_REG_XOUT0, buf, 7);

    parse_xyz_raw(buf, raw, dev->use_18bit);

    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_ReadMag(mmc5983ma_dev_t* dev,
                                  mmc5983ma_data_t* data) {
    mmc5983ma_err_t r;
    mmc5983ma_raw_data_t raw;
    float counts_per_g;
    float null_field;

    if (!dev || !data)
        return MMC_ERR_PARAM;

    r = MMC5983MA_ReadMagRaw(dev, &raw);
    if (r)
        return r;

    if (dev->use_18bit) {
        counts_per_g = 16384.0f;
        null_field = (float)MMC_NULL_FIELD_18BIT;
    } else {
        counts_per_g = 4096.0f;
        null_field = (float)MMC_NULL_FIELD_16BIT;
    }

    data->x = ((float)raw.x - null_field) / counts_per_g;
    data->y = ((float)raw.y - null_field) / counts_per_g;
    data->z = ((float)raw.z - null_field) / counts_per_g;

    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  连续模式读取（不触发新测量）
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_IsMagDataReady(mmc5983ma_dev_t* dev,
                                         uint8_t* ready) {
    uint8_t status;
    if (!dev || !ready)
        return MMC_ERR_PARAM;

    status = reg_read(dev, MMC_REG_STATUS);
    *ready = (status & MMC_STATUS_MEAS_M_DONE) ? 1 : 0;

    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_ReadMagRawOnly(mmc5983ma_dev_t* dev,
                                         mmc5983ma_raw_data_t* raw) {
    uint8_t buf[7];

    if (!dev || !raw)
        return MMC_ERR_PARAM;

    reg_read_burst(dev, MMC_REG_XOUT0, buf, 7);
    parse_xyz_raw(buf, raw, dev->use_18bit);

    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_ReadMagOnly(mmc5983ma_dev_t* dev,
                                      mmc5983ma_data_t* data) {
    mmc5983ma_err_t r;
    mmc5983ma_raw_data_t raw;
    float counts_per_g;
    float null_field;

    if (!dev || !data)
        return MMC_ERR_PARAM;

    r = MMC5983MA_ReadMagRawOnly(dev, &raw);
    if (r)
        return r;

    if (dev->use_18bit) {
        counts_per_g = 16384.0f;
        null_field = (float)MMC_NULL_FIELD_18BIT;
    } else {
        counts_per_g = 4096.0f;
        null_field = (float)MMC_NULL_FIELD_16BIT;
    }

    data->x = ((float)raw.x - null_field) / counts_per_g;
    data->y = ((float)raw.y - null_field) / counts_per_g;
    data->z = ((float)raw.z - null_field) / counts_per_g;

    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  温度
 *
 *  Tout (0x07): 8-bit unsigned
 *  范围 -75°C ~ +125°C
 *  0x00 = -75°C, 每 LSB ≈ 0.8°C
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_ReadTemp(mmc5983ma_dev_t* dev,
                                   float* temperature_c) {
    mmc5983ma_err_t r;
    uint8_t tout;

    if (!dev || !temperature_c)
        return MMC_ERR_PARAM;

    /* 触发温度测量 */
    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_TM_T);

    r = wait_temp_done(dev);
    if (r)
        return r;

    tout = reg_read(dev, MMC_REG_TOUT);

    *temperature_c = -75.0f + (float)tout * 0.8f;

    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  航向角
 *  heading = atan2(Y, X), 结果 0~360°
 *  假设传感器水平放置
 * ═══════════════════════════════════════════════════ */

float MMC5983MA_CalcHeading(const mmc5983ma_data_t* mag) {
    float heading;

    if (!mag)
        return 0.0f;

    heading = atan2f(mag->y, mag->x) * 180.0f / 3.14159265f;
    if (heading < 0.0f)
        heading += 360.0f;

    return heading;
}

/* ═══════════════════════════════════════════════════
 *  SET/RESET 补偿测量（消除电桥偏移）
 *
 *  流程:
 *    1. SET → 测量 → Output1 = +H + Offset
 *    2. RESET → 测量 → Output2 = -H + Offset
 *    3. H = (Output1 - Output2) / 2
 *    4. Offset = (Output1 + Output2) / 2
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_ReadMagCompensated(mmc5983ma_dev_t* dev,
                                             mmc5983ma_data_t* data,
                                             mmc5983ma_data_t* offset) {
    mmc5983ma_err_t r;
    mmc5983ma_data_t d1, d2;
    float counts_per_g;
    float null_field;
    float ox, oy, oz; /* 原始 offset (counts) */
    mmc5983ma_raw_data_t r1, r2;

    if (!dev || !data)
        return MMC_ERR_PARAM;

    if (dev->use_18bit) {
        counts_per_g = 16384.0f;
        null_field = (float)MMC_NULL_FIELD_18BIT;
    } else {
        counts_per_g = 4096.0f;
        null_field = (float)MMC_NULL_FIELD_16BIT;
    }

    /* 1. SET */
    r = MMC5983MA_Set(dev);
    if (r)
        return r;

    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_TM_M);
    r = wait_mag_done(dev);
    if (r)
        return r;

    {
        uint8_t buf[7];
        reg_read_burst(dev, MMC_REG_XOUT0, buf, 7);
        parse_xyz_raw(buf, &r1, dev->use_18bit);
    }

    /* 2. RESET */
    r = MMC5983MA_Reset(dev);
    if (r)
        return r;

    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_TM_M);
    r = wait_mag_done(dev);
    if (r)
        return r;

    {
        uint8_t buf[7];
        reg_read_burst(dev, MMC_REG_XOUT0, buf, 7);
        parse_xyz_raw(buf, &r2, dev->use_18bit);
    }

    /* 3. H = (SET_measured - RESET_measured) / 2 */
    /*    Offset = (SET_measured + RESET_measured) / 2 */
    ox = ((float)r1.x + (float)r2.x) / 2.0f;
    oy = ((float)r1.y + (float)r2.y) / 2.0f;
    oz = ((float)r1.z + (float)r2.z) / 2.0f;

    data->x = ((float)r1.x - (float)r2.x) / (2.0f * counts_per_g);
    data->y = ((float)r1.y - (float)r2.y) / (2.0f * counts_per_g);
    data->z = ((float)r1.z - (float)r2.z) / (2.0f * counts_per_g);

    if (offset) {
        offset->x = (ox - null_field) / counts_per_g;
        offset->y = (oy - null_field) / counts_per_g;
        offset->z = (oz - null_field) / counts_per_g;
    }

    return MMC_OK;
}
