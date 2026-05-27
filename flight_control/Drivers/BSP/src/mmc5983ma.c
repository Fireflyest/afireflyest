/**
 * @file mmc5983ma.c
 * @brief MMC5983MA 磁传感器驱动
 *        参考 Tlera Corp + SparkFun 开源库
 *        不使用 Shadow Register，不使用 Auto_SR_EN
 *        CTRL0 所有操作均为直接写入（Write-Only 寄存器）
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
 *  底层 SPI 读写
 * ═══════════════════════════════════════════════════ */

static uint8_t reg_read(const mmc5983ma_dev_t* dev, uint8_t reg) {
    uint8_t val = 0;
    SPI_Sensor_ReadBytes(dev->sensor_id, reg, &val, 1);
    return val;
}

static void reg_write(const mmc5983ma_dev_t* dev, uint8_t reg, uint8_t val) {
    SPI_Sensor_WriteBytes(dev->sensor_id, reg, &val, 1);
}

static void reg_read_burst(const mmc5983ma_dev_t* dev, uint8_t reg, uint8_t* buf, uint16_t len) {
    SPI_Sensor_ReadBytes(dev->sensor_id, reg, buf, len);
}

/* ═══════════════════════════════════════════════════
 *  前向声明
 * ═══════════════════════════════════════════════════ */

static void parse_xyz_raw(const uint8_t buf[7],
                          mmc5983ma_raw_data_t* raw,
                          uint8_t use_18bit);

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
    reg_write(dev, MMC_REG_CTRL1, MMC_CTRL1_SW_RST);
    delay_ms(15);
    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_Init(mmc5983ma_dev_t* dev) {
    uint8_t pid;
    uint8_t status;

    if (!dev)
        return MMC_ERR_PARAM;

    /* 软复位 */
    MMC5983MA_SoftReset(dev);

    /* dummy read */
    (void)reg_read(dev, MMC_REG_PRODUCT_ID);

    /* 验证 Product ID */
    pid = reg_read(dev, MMC_REG_PRODUCT_ID);
    if (pid != MMC_PRODUCT_ID)
        return MMC_ERR_PRODUCT_ID;

    /* 读 OTP */
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

    /* CTRL1: 带宽 BW=00 */
    dev->bw = MMC_BW_00;
    reg_write(dev, MMC_REG_CTRL1, (uint8_t)dev->bw);

    /* CTRL2: 关闭连续模式 */
    reg_write(dev, MMC_REG_CTRL2, 0x00);

    /* CTRL0: 清零（不使用 Auto_SR_EN，采用手动 SET/RESET） */
    reg_write(dev, MMC_REG_CTRL0, 0x00);

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
    reg_write(dev, MMC_REG_CTRL1, (uint8_t)bw);
    dev->bw = bw;
    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  SET / RESET（直接写入，参考 Tlera Corp）
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_Set(mmc5983ma_dev_t* dev) {
    if (!dev)
        return MMC_ERR_PARAM;
    reg_write(dev, MMC_REG_CTRL0, 0x08); /* SET */
    delay_ms(1);
    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_Reset(mmc5983ma_dev_t* dev) {
    if (!dev)
        return MMC_ERR_PARAM;
    reg_write(dev, MMC_REG_CTRL0, 0x10); /* RESET */
    delay_ms(1);
    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  等待
 * ═══════════════════════════════════════════════════ */

static const uint8_t meas_time_ms[4] = {10, 6, 4, 2};

static mmc5983ma_err_t wait_mag_done(mmc5983ma_dev_t* dev) {
    uint32_t timeout = (uint32_t)meas_time_ms[dev->bw & 0x03] + 5;
    uint32_t t = 0;
    do {
        if (reg_read(dev, MMC_REG_STATUS) & MMC_STATUS_MEAS_M_DONE)
            return MMC_OK;
        delay_ms(1);
    } while (++t < timeout);
    return MMC_ERR_TIMEOUT;
}

static mmc5983ma_err_t wait_temp_done(mmc5983ma_dev_t* dev) {
    uint32_t t = 0;
    do {
        if (reg_read(dev, MMC_REG_STATUS) & MMC_STATUS_MEAS_T_DONE)
            return MMC_OK;
        delay_ms(1);
    } while (++t < 20);
    return MMC_ERR_TIMEOUT;
}

/* ═══════════════════════════════════════════════════
 *  XYZ 解析
 * ═══════════════════════════════════════════════════ */

static void parse_xyz_raw(const uint8_t buf[7],
                          mmc5983ma_raw_data_t* raw,
                          uint8_t use_18bit) {
    uint32_t x = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | (((uint32_t)buf[6] >> 6) & 0x03);
    uint32_t y = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | (((uint32_t)buf[6] >> 4) & 0x03);
    uint32_t z = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | (((uint32_t)buf[6] >> 2) & 0x03);

    if (!use_18bit) {
        x >>= 2;
        y >>= 2;
        z >>= 2;
    }

    raw->x = x;
    raw->y = y;
    raw->z = z;
}

/* ═══════════════════════════════════════════════════
 *  单次磁场测量（参考 Tlera Corp selfTest 流程）
 *
 *  1. SET  → 磁化 AMR 桥
 *  2. TM_M → 触发测量
 *  3. 等待 Meas_M_Done
 *  4. 读 7 字节
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_ReadMagRaw(mmc5983ma_dev_t* dev,
                                     mmc5983ma_raw_data_t* raw) {
    uint8_t buf[7];

    if (!dev || !raw)
        return MMC_ERR_PARAM;

    /* 1. SET: 磁化 AMR 桥 */
    reg_write(dev, MMC_REG_CTRL0, 0x08);
    delay_ms(1);

    /* 2. TM_M: 触发磁场测量 */
    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_TM_M);

    /* 3. 等待完成 */
    {
        mmc5983ma_err_t r = wait_mag_done(dev);
        if (r)
            return r;
    }

    /* 4. burst read */
    reg_read_burst(dev, MMC_REG_XOUT0, buf, 7);
    parse_xyz_raw(buf, raw, dev->use_18bit);

    return MMC_OK;
}

mmc5983ma_err_t MMC5983MA_ReadMag(mmc5983ma_dev_t* dev,
                                  mmc5983ma_data_t* data) {
    mmc5983ma_raw_data_t raw;
    mmc5983ma_err_t r;
    float cpg, nf;

    if (!dev || !data)
        return MMC_ERR_PARAM;

    r = MMC5983MA_ReadMagRaw(dev, &raw);
    if (r)
        return r;

    if (dev->use_18bit) {
        cpg = 16384.0f;
        nf = (float)MMC_NULL_FIELD_18BIT;
    } else {
        cpg = 4096.0f;
        nf = (float)MMC_NULL_FIELD_16BIT;
    }

    data->x = ((float)raw.x - nf) / cpg;
    data->y = ((float)raw.y - nf) / cpg;
    data->z = ((float)raw.z - nf) / cpg;
    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  温度
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_ReadTemp(mmc5983ma_dev_t* dev,
                                   float* temperature_c) {
    if (!dev || !temperature_c)
        return MMC_ERR_PARAM;

    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_TM_T);
    wait_temp_done(dev);

    uint8_t tout = reg_read(dev, MMC_REG_TOUT);
    *temperature_c = -75.0f + (float)tout * (200.0f / 255.0f);
    return MMC_OK;
}

/* ═══════════════════════════════════════════════════
 *  连续模式读取
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_IsMagDataReady(mmc5983ma_dev_t* dev,
                                         uint8_t* ready) {
    if (!dev || !ready)
        return MMC_ERR_PARAM;
    *ready = (reg_read(dev, MMC_REG_STATUS) & MMC_STATUS_MEAS_M_DONE) ? 1 : 0;
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

/* ═══════════════════════════════════════════════════
 *  航向角
 * ═══════════════════════════════════════════════════ */

float MMC5983MA_CalcHeading(const mmc5983ma_data_t* mag) {
    if (!mag)
        return 0.0f;
    float h = atan2f(mag->y, mag->x) * 180.0f / 3.14159265f;
    if (h < 0.0f)
        h += 360.0f;
    return h;
}

/* ═══════════════════════════════════════════════════
 *  补偿测量（参考 Tlera Corp selfTest 流程）
 *  SET → 测量 → RESET → 测量
 *  H = (SET - RESET) / 2
 * ═══════════════════════════════════════════════════ */

mmc5983ma_err_t MMC5983MA_ReadMagCompensated(mmc5983ma_dev_t* dev,
                                             mmc5983ma_data_t* data,
                                             mmc5983ma_data_t* offset) {
    mmc5983ma_raw_data_t r1, r2;
    float cpg, nf;
    uint8_t buf[7];

    if (!dev || !data)
        return MMC_ERR_PARAM;

    if (dev->use_18bit) {
        cpg = 16384.0f;
        nf = 131072.0f;
    } else {
        cpg = 4096.0f;
        nf = 32768.0f;
    }

    /* 1. SET → 测量 */
    reg_write(dev, MMC_REG_CTRL0, 0x08);
    delay_ms(1);
    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_TM_M);
    {
        mmc5983ma_err_t r = wait_mag_done(dev);
        if (r)
            return r;
    }
    reg_read_burst(dev, MMC_REG_XOUT0, buf, 7);
    parse_xyz_raw(buf, &r1, dev->use_18bit);

    /* 2. RESET → 测量 */
    reg_write(dev, MMC_REG_CTRL0, 0x10);
    delay_ms(1);
    reg_write(dev, MMC_REG_CTRL0, MMC_CTRL0_TM_M);
    {
        mmc5983ma_err_t r = wait_mag_done(dev);
        if (r)
            return r;
    }
    reg_read_burst(dev, MMC_REG_XOUT0, buf, 7);
    parse_xyz_raw(buf, &r2, dev->use_18bit);

    /* 3. 计算 */
    data->x = ((float)r1.x - (float)r2.x) / (2.0f * cpg);
    data->y = ((float)r1.y - (float)r2.y) / (2.0f * cpg);
    data->z = ((float)r1.z - (float)r2.z) / (2.0f * cpg);

    if (offset) {
        offset->x = (((float)r1.x + (float)r2.x) / 2.0f - nf) / cpg;
        offset->y = (((float)r1.y + (float)r2.y) / 2.0f - nf) / cpg;
        offset->z = (((float)r1.z + (float)r2.z) / 2.0f - nf) / cpg;
    }

    return MMC_OK;
}
