/**
 * @file bmp580.c
 * @brief BMP580 驱动
 */

#include "bmp580.h"
#include <math.h>

/* ═══════════════════════════════════════════════════
 *  内部延时
 * ═══════════════════════════════════════════════════ */

static bmp580_delay_fn_t s_delay = NULL;

void BMP580_SetDelay(bmp580_delay_fn_t fn) {
    s_delay = fn;
}

static void delay_ms(uint32_t ms) {
    if (s_delay)
        s_delay(ms);
}

/* ═══════════════════════════════════════════════════
 *  底层寄存器读写
 * ═══════════════════════════════════════════════════ */

/** 读单个寄存器，返回字节值 */
static uint8_t reg_read(const bmp580_dev_t* dev, uint8_t reg) {
    uint8_t val = 0;
    SPI_Sensor_ReadBytes(dev->sensor_id, reg, &val, 1);
    return val;
}

/** 写单个寄存器 */
static void reg_write(const bmp580_dev_t* dev, uint8_t reg, uint8_t val) {
    SPI_Sensor_WriteBytes(dev->sensor_id, reg, &val, 1);
}

/** 连续读多个寄存器 */
static void reg_read_burst(const bmp580_dev_t* dev,
                           uint8_t reg,
                           uint8_t* buf,
                           uint16_t len) {
    SPI_Sensor_ReadBytes(dev->sensor_id, reg, buf, len);
}

/** 连续写多个寄存器 */
static void reg_write_burst(const bmp580_dev_t* dev,
                            uint8_t reg,
                            const uint8_t* buf,
                            uint16_t len) {
    SPI_Sensor_WriteBytes(dev->sensor_id, reg, buf, len);
}

/** 读-改-写 */
static void reg_rmw(const bmp580_dev_t* dev,
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

bmp580_err_t BMP580_WhoAmI(bmp580_dev_t* dev, uint8_t* chip_id) {
    if (!dev || !chip_id)
        return BMP580_ERR_PARAM;
    *chip_id = reg_read(dev, BMP580_REG_CHIP_ID);
    return BMP580_OK;
}

bmp580_err_t BMP580_SoftReset(bmp580_dev_t* dev) {
    if (!dev)
        return BMP580_ERR_PARAM;
    reg_write(dev, BMP580_REG_CMD, BMP580_CMD_SOFT_RESET);
    delay_ms(2);
    return BMP580_OK;
}

bmp580_err_t BMP580_Init(bmp580_dev_t* dev) {
    uint8_t chip_id, status;

    if (!dev)
        return BMP580_ERR_PARAM;

    delay_ms(2); /* t_powerup */
    BMP580_SoftReset(dev);
    delay_ms(10);

    {
        uint8_t chip_id;
        bmp580_err_t err;
        for (uint32_t t = 0; t < 100; t++) {
            delay_ms(5);
            err = BMP580_WhoAmI(dev, &chip_id);
            if (err == BMP580_OK)
                break;
        }
        if (err != BMP580_OK)
            return err;
    }

    status = reg_read(dev, BMP580_REG_STATUS);
    if (!(status & 0x01))
        return BMP580_ERR_TIMEOUT; /* nvm_rdy */
    if (status & 0x04)
        return BMP580_ERR_NVM; /* nvm_err */

    (void)reg_read(dev, BMP580_REG_INT_STATUS); /* 清 POR */

    dev->osr_p = BMP580_OSR_1X;
    dev->osr_t = BMP580_OSR_1X;
    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  电源模式
 *  ODR_CONFIG(0x37): [7:3]=odr, [2:0]=pwr_mode
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_SetPowerMode(bmp580_dev_t* dev, bmp580_mode_t mode) {
    if (!dev)
        return BMP580_ERR_PARAM;

    reg_rmw(dev, BMP580_REG_ODR_CONFIG, 0x07, BMP580_MODE_STANDBY);
    delay_ms(3);
    reg_rmw(dev, BMP580_REG_ODR_CONFIG, 0x07, (uint8_t)mode);
    delay_ms(3);
    return BMP580_OK;
}

bmp580_err_t BMP580_GetPowerMode(bmp580_dev_t* dev, bmp580_mode_t* mode) {
    if (!dev || !mode)
        return BMP580_ERR_PARAM;
    *mode = (bmp580_mode_t)(reg_read(dev, BMP580_REG_ODR_CONFIG) & 0x07);
    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  测量配置
 *  OSR_CONFIG(0x36): [6:4]=osr_t, [3:1]=osr_p, [0]=press_en
 *  ODR_CONFIG(0x37): [7:3]=odr
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigOSR(bmp580_dev_t* dev,
                              bmp580_osr_t osr_p,
                              bmp580_osr_t osr_t) {
    uint8_t val;
    if (!dev)
        return BMP580_ERR_PARAM;

    val = reg_read(dev, BMP580_REG_OSR_CONFIG) & 0x01; /* 保留 press_en */
    val |= (uint8_t)(((osr_t & 0x07) << 4) | ((osr_p & 0x07) << 1));
    reg_write(dev, BMP580_REG_OSR_CONFIG, val);

    dev->osr_p = osr_p;
    dev->osr_t = osr_t;
    return BMP580_OK;
}

bmp580_err_t BMP580_EnablePressure(bmp580_dev_t* dev, uint8_t enable) {
    if (!dev)
        return BMP580_ERR_PARAM;
    reg_rmw(dev, BMP580_REG_OSR_CONFIG, 0x01, enable ? 0x01 : 0x00);
    return BMP580_OK;
}

bmp580_err_t BMP580_ConfigODR(bmp580_dev_t* dev, bmp580_odr_t odr) {
    if (!dev)
        return BMP580_ERR_PARAM;
    reg_rmw(dev, BMP580_REG_ODR_CONFIG, 0xF8, (uint8_t)((odr & 0x1F) << 3));
    return BMP580_OK;
}

bmp580_err_t BMP580_ConfigMeasurement(bmp580_dev_t* dev,
                                      bmp580_osr_t osr_p,
                                      bmp580_osr_t osr_t,
                                      bmp580_odr_t odr,
                                      uint8_t press_enable) {
    bmp580_err_t r;
    if (!dev)
        return BMP580_ERR_PARAM;

    r = BMP580_ConfigOSR(dev, osr_p, osr_t);
    if (r)
        return r;
    r = BMP580_EnablePressure(dev, press_enable);
    if (r)
        return r;
    return BMP580_ConfigODR(dev, odr);
}

/* ═══════════════════════════════════════════════════
 *  IIR 滤波器
 *  DSP_IIR(0x31): [6:4]=iir_p, [2:0]=iir_t
 *  DSP_CONFIG(0x30): bit0=iir_flush_forced_en
 *                    bit1=swdw_sei_ir_t
 *                    bit3=swdw_sei_ir_p
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigIIR(bmp580_dev_t* dev,
                              bmp580_iir_t iir_p,
                              bmp580_iir_t iir_t,
                              uint8_t flush_en) {
    uint8_t val;
    if (!dev)
        return BMP580_ERR_PARAM;

    val = (uint8_t)(((iir_p & 0x07) << 4) | (iir_t & 0x07));
    reg_write(dev, BMP580_REG_DSP_IIR, val);
    reg_rmw(dev, BMP580_REG_DSP_CONFIG, 0x01, flush_en ? 0x01 : 0x00);
    return BMP580_OK;
}

bmp580_err_t BMP580_SetDataSrc(bmp580_dev_t* dev,
                               uint8_t src_t,
                               uint8_t src_p) {
    if (!dev)
        return BMP580_ERR_PARAM;
    reg_rmw(dev, BMP580_REG_DSP_CONFIG, (1u << 1), (src_t & 1u) << 1);
    reg_rmw(dev, BMP580_REG_DSP_CONFIG, (1u << 3), (src_p & 1u) << 3);
    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  数据读取
 *
 *  从 TEMP_XLSB(0x1D) 连续读 6 字节：
 *    buf[0]=T_XLSB, buf[1]=T_LSB, buf[2]=T_MSB,
 *    buf[3]=P_XLSB, buf[4]=P_LSB, buf[5]=P_MSB
 *
 *  温度: T[°C] = raw24 / 65536
 *  压力: P[Pa] = raw24 / 64
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ReadData(bmp580_dev_t* dev, bmp580_data_t* data) {
    uint8_t buf[6];

    if (!dev || !data)
        return BMP580_ERR_PARAM;

    reg_read_burst(dev, BMP580_REG_TEMP_XLSB, buf, 6);

    data->raw_temp = ((int32_t)(int8_t)buf[2] << 16) |
                     ((int32_t)buf[1] << 8) |
                     ((int32_t)buf[0]);

    data->raw_press = ((int32_t)(int8_t)buf[5] << 16) |
                      ((int32_t)buf[4] << 8) |
                      ((int32_t)buf[3]);

    data->temperature_deg = (float)data->raw_temp / 65536.0f;
    data->pressure_pa = (float)data->raw_press / 64.0f;

    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  FORCED 模式
 * ═══════════════════════════════════════════════════ */

static const uint16_t osr_time_ms[8] = {
    2, 3, 4, 7, 12, 24, 45, 90};

bmp580_err_t BMP580_ForcedMeasure(bmp580_dev_t* dev, bmp580_data_t* data) {
    bmp580_mode_t mode;
    uint32_t timeout, elapsed = 0;

    if (!dev || !data)
        return BMP580_ERR_PARAM;

    timeout = (uint32_t)osr_time_ms[dev->osr_p & 7] + (uint32_t)osr_time_ms[dev->osr_t & 7] + 10;

    reg_rmw(dev, BMP580_REG_ODR_CONFIG, 0x07, BMP580_MODE_FORCED);

    do {
        delay_ms(1);
        elapsed++;
        mode = (bmp580_mode_t)(reg_read(dev, BMP580_REG_ODR_CONFIG) & 0x07);
        if (mode == BMP580_MODE_STANDBY)
            break;
    } while (elapsed < timeout);

    if (elapsed >= timeout)
        return BMP580_ERR_TIMEOUT;

    return BMP580_ReadData(dev, data);
}

/* ═══════════════════════════════════════════════════
 *  FIFO
 *  FIFO_CONFIG(0x16): [5:1]=threshold, [0]=mode
 *  FIFO_SEL(0x18):    [4:2]=dec_sel, [1:0]=frame_sel
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigFIFO(bmp580_dev_t* dev,
                               bmp580_fifo_frame_t frame_sel,
                               bmp580_fifo_mode_t mode,
                               uint8_t dec_sel,
                               uint8_t threshold) {
    uint8_t val;
    if (!dev)
        return BMP580_ERR_PARAM;
    if (threshold > 31 || dec_sel > 7)
        return BMP580_ERR_PARAM;

    val = (uint8_t)(((threshold & 0x1F) << 1) | (mode & 0x01));
    reg_write(dev, BMP580_REG_FIFO_CONFIG, val);

    val = (uint8_t)(((dec_sel & 0x07) << 2) | (frame_sel & 0x03));
    reg_write(dev, BMP580_REG_FIFO_SEL, val);

    return BMP580_OK;
}

bmp580_err_t BMP580_GetFIFOCount(bmp580_dev_t* dev, uint8_t* count) {
    if (!dev || !count)
        return BMP580_ERR_PARAM;
    *count = reg_read(dev, BMP580_REG_FIFO_COUNT) & 0x3F;
    return BMP580_OK;
}

bmp580_err_t BMP580_ReadFIFO(bmp580_dev_t* dev,
                             uint8_t* buf,
                             uint32_t byte_count) {
    if (!dev || !buf || !byte_count)
        return BMP580_ERR_PARAM;
    reg_read_burst(dev, BMP580_REG_FIFO_DATA, buf, (uint16_t)byte_count);
    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  中断
 *  INT_CONFIG(0x14): [3]=mode, [2]=od, [1]=pol, [0]=en
 *  INT_SOURCE(0x15): [3:0] source bitmap
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigInt(bmp580_dev_t* dev,
                              uint8_t src_bitmap,
                              uint8_t active_high,
                              uint8_t open_drain,
                              uint8_t latched) {
    uint8_t val;
    if (!dev)
        return BMP580_ERR_PARAM;

    val = (uint8_t)((latched << 3) | (open_drain << 2) |
                    (active_high << 1) | 0x01);
    reg_write(dev, BMP580_REG_INT_CONFIG, val);
    reg_write(dev, BMP580_REG_INT_SOURCE, src_bitmap & 0x0F);
    return BMP580_OK;
}

bmp580_err_t BMP580_GetIntStatus(bmp580_dev_t* dev, uint8_t* status) {
    if (!dev || !status)
        return BMP580_ERR_PARAM;
    *status = reg_read(dev, BMP580_REG_INT_STATUS); /* clear-on-read */
    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  NVM
 * ═══════════════════════════════════════════════════ */

static bmp580_err_t nvm_wait_ready(const bmp580_dev_t* dev, uint32_t timeout_ms) {
    uint32_t t = 0;
    do {
        if (reg_read(dev, BMP580_REG_STATUS) & 0x01)
            return BMP580_OK;
        delay_ms(1);
    } while (++t < timeout_ms);
    return BMP580_ERR_TIMEOUT;
}

bmp580_err_t BMP580_NVMRead(bmp580_dev_t* dev,
                            uint8_t address,
                            uint16_t* data) {
    bmp580_err_t r;
    uint8_t lsb, msb;

    if (!dev || !data)
        return BMP580_ERR_PARAM;
    if (address < BMP580_NVM_USER_ADDR_MIN ||
        address > BMP580_NVM_USER_ADDR_MAX)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    r = nvm_wait_ready(dev, 100);
    if (r)
        return r;

    reg_write(dev, BMP580_REG_NVM_ADDR, address & 0x3F);
    reg_write(dev, BMP580_REG_CMD, BMP580_CMD_NVM_TRIGGER);
    reg_write(dev, BMP580_REG_CMD, BMP580_CMD_NVM_READ);

    delay_ms(1);
    r = nvm_wait_ready(dev, 100);
    if (r)
        return r;

    lsb = reg_read(dev, BMP580_REG_NVM_DATA_LSB);
    msb = reg_read(dev, BMP580_REG_NVM_DATA_MSB);
    *data = (uint16_t)((msb << 8) | lsb);

    if (reg_read(dev, BMP580_REG_STATUS) & 0x04)
        return BMP580_ERR_NVM;

    return BMP580_OK;
}

bmp580_err_t BMP580_NVMWrite(bmp580_dev_t* dev,
                             uint8_t address,
                             uint16_t data) {
    bmp580_err_t r;

    if (!dev)
        return BMP580_ERR_PARAM;
    if (address < BMP580_NVM_USER_ADDR_MIN ||
        address > BMP580_NVM_USER_ADDR_MAX)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    r = nvm_wait_ready(dev, 100);
    if (r)
        return r;

    reg_write(dev, BMP580_REG_NVM_ADDR, (address & 0x3F) | 0x80);
    reg_write(dev, BMP580_REG_NVM_DATA_LSB, (uint8_t)(data & 0xFF));
    reg_write(dev, BMP580_REG_NVM_DATA_MSB, (uint8_t)(data >> 8));
    reg_write(dev, BMP580_REG_CMD, BMP580_CMD_NVM_TRIGGER);
    reg_write(dev, BMP580_REG_CMD, BMP580_CMD_NVM_PROG);

    r = nvm_wait_ready(dev, 50);
    if (r)
        return r;

    if (reg_read(dev, BMP580_REG_STATUS) & 0x04)
        return BMP580_ERR_NVM;

    reg_write(dev, BMP580_REG_NVM_ADDR, address & 0x3F);
    return BMP580_OK;
}

bmp580_err_t BMP580_ReadUID(bmp580_dev_t* dev, uint64_t* uid) {
    bmp580_err_t r;
    uint16_t nvm[4]; /* 0x23, 0x24, 0x25, 0x26 */

    if (!dev || !uid)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    for (int i = 0; i < 4; i++) {
        uint8_t addr = 0x23 + (uint8_t)i;
        r = nvm_wait_ready(dev, 100);
        if (r)
            return r;

        reg_write(dev, BMP580_REG_NVM_ADDR, addr & 0x3F);
        reg_write(dev, BMP580_REG_CMD, BMP580_CMD_NVM_TRIGGER);
        reg_write(dev, BMP580_REG_CMD, BMP580_CMD_NVM_READ);

        delay_ms(1);
        r = nvm_wait_ready(dev, 100);
        if (r)
            return r;

        nvm[i] = (uint16_t)((reg_read(dev, BMP580_REG_NVM_DATA_MSB) << 8) |
                            reg_read(dev, BMP580_REG_NVM_DATA_LSB));
    }

    *uid = (((uint64_t)(nvm[3] & 0x00FF)) << 40) | /* 0x26 */
           (((uint64_t)nvm[2]) << 24) |            /* 0x25 */
           (((uint64_t)nvm[1]) << 8) |             /* 0x24 */
           (((uint64_t)(nvm[0] & 0xFF00)) >> 8);   /* 0x23 */

    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  高度计算
 * ═══════════════════════════════════════════════════ */

#define ISA_P0 101325.0f
#define ISA_T0 288.15f
#define ISA_L 0.0065f
#define ISA_G 9.80665f
#define ISA_M 0.0289644f
#define ISA_R 8.31447f
#define ISA_EXP (-ISA_G * ISA_M / (ISA_R * ISA_L)) /* ≈ -5.2559 */

float BMP580_PressureToAltitude(float press_pa, float sea_level_pa) {
    if (sea_level_pa <= 0.0f)
        sea_level_pa = ISA_P0;
    return 44330.0f * (1.0f - powf(press_pa / sea_level_pa, 1.0f / 5.255f));
}

float BMP580_PressureToAltitudeWithTemp(float press_pa,
                                        float temperature_c,
                                        float sea_level_pa) {
    float t0k;
    if (sea_level_pa <= 0.0f)
        sea_level_pa = ISA_P0;
    t0k = temperature_c + 273.15f;
    return (t0k / ISA_L) * (powf(press_pa / sea_level_pa, ISA_EXP) - 1.0f);
}

float BMP580_AltitudeToSeaLevelPressure(float press_pa,
                                        float temperature_c,
                                        float altitude_m) {
    float t0k = temperature_c + 273.15f;
    return press_pa * powf(1.0f - ISA_L * altitude_m / t0k, ISA_EXP);
}
