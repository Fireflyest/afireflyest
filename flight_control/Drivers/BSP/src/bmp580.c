/**
 * @file bmp580.c
 * @brief BMP580 驱动（参考 Bosch 官方实现）
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

static uint8_t reg_read(const bmp580_dev_t* dev, uint8_t reg) {
    uint8_t val = 0;
    SPI_Sensor_ReadBytes(dev->sensor_id, reg, &val, 1);
    return val;
}

static void reg_write(const bmp580_dev_t* dev, uint8_t reg, uint8_t val) {
    SPI_Sensor_WriteBytes(dev->sensor_id, reg, &val, 1);
}

static void reg_read_burst(const bmp580_dev_t* dev,
                           uint8_t reg,
                           uint8_t* buf,
                           uint16_t len) {
    SPI_Sensor_ReadBytes(dev->sensor_id, reg, buf, len);
}

static void reg_write_burst(const bmp580_dev_t* dev,
                            uint8_t reg,
                            const uint8_t* buf,
                            uint16_t len) {
    SPI_Sensor_WriteBytes(dev->sensor_id, reg, buf, len);
}

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
    *chip_id = reg_read(dev, BMP5_REG_CHIP_ID);
    return BMP580_OK;
}

bmp580_err_t BMP580_SoftReset(bmp580_dev_t* dev) {
    uint8_t por_status;

    if (!dev)
        return BMP580_ERR_PARAM;

    reg_write(dev, BMP5_REG_CMD, BMP5_SOFT_RESET_CMD);
    delay_ms(2);

    /* SPI dummy read after soft reset */
    (void)reg_read(dev, BMP5_REG_CHIP_ID);

    /* 验证 POR 状态 */
    por_status = reg_read(dev, BMP5_REG_INT_STATUS);
    if (!(por_status & BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE))
        return BMP580_ERR_POR;

    return BMP580_OK;
}

bmp580_err_t BMP580_Init(bmp580_dev_t* dev) {
    uint8_t chip_id;
    uint8_t nvm_status;
    uint8_t por_status;

    if (!dev)
        return BMP580_ERR_PARAM;

    dev->chip_id = 0;
    dev->osr_p = BMP580_OSR_1X;
    dev->osr_t = BMP580_OSR_1X;

    delay_ms(2); /* t_powerup */

    /* ★ SPI dummy read（建立 SPI 通信状态） */
    (void)reg_read(dev, BMP5_REG_CHIP_ID);

    /* 读取 Chip ID */
    chip_id = reg_read(dev, BMP5_REG_CHIP_ID);

    /* 验证 Chip ID */
    if (chip_id != BMP5_CHIP_ID_PRIM && chip_id != BMP5_CHIP_ID_SEC)
        return BMP580_ERR_CHIP_ID;

    dev->chip_id = chip_id;

    /* ★ 上电检查：NVM 就绪 + POR 状态 */
    nvm_status = reg_read(dev, BMP5_REG_STATUS);
    if (!(nvm_status & BMP5_INT_NVM_RDY))
        return BMP580_ERR_POWER_UP;
    if (nvm_status & BMP5_INT_NVM_ERR)
        return BMP580_ERR_NVM;

    por_status = reg_read(dev, BMP5_REG_INT_STATUS);
    if (!(por_status & BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE))
        return BMP580_ERR_POR;

    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  电源模式
 * ═══════════════════════════════════════════════════ */

/** 底层写入模式（不检查当前状态） */
static bmp580_err_t set_power_mode_raw(const bmp580_dev_t* dev,
                                       bmp580_mode_t mode) {
    uint8_t reg_data = reg_read(dev, BMP5_REG_ODR_CONFIG);

    /* 关闭 deep standby */
    reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_DEEP_DISABLE, BMP5_DEEP_DISABLED);
    /* 写入电源模式 */
    reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_POWERMODE, (uint8_t)mode);

    reg_write(dev, BMP5_REG_ODR_CONFIG, reg_data);
    return BMP580_OK;
}

bmp580_err_t BMP580_GetPowerMode(bmp580_dev_t* dev, bmp580_mode_t* mode) {
    if (!dev || !mode)
        return BMP580_ERR_PARAM;

    uint8_t reg_data = reg_read(dev, BMP5_REG_ODR_CONFIG);
    *mode = (bmp580_mode_t)(BMP5_GET_BITS_POS_0(reg_data, BMP5_POWERMODE));
    return BMP580_OK;
}

bmp580_err_t BMP580_SetPowerMode(bmp580_dev_t* dev, bmp580_mode_t mode) {
    bmp580_mode_t current;

    if (!dev)
        return BMP580_ERR_PARAM;

    BMP580_GetPowerMode(dev, &current);

    /* 切换前必须先回 Standby */
    if (current != BMP580_MODE_STANDBY) {
        set_power_mode_raw(dev, BMP580_MODE_STANDBY);
        delay_ms(3);
    }

    if (mode != BMP580_MODE_STANDBY) {
        set_power_mode_raw(dev, mode);
        delay_ms(3);
    }

    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  测量配置
 *  OSR_CONFIG(0x36): [6:4]=osr_t, [3:1]=osr_p, [0]=press_en
 *  ODR_CONFIG(0x37):  [7:3]=odr
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigOSR(bmp580_dev_t* dev,
                              bmp580_osr_t osr_p,
                              bmp580_osr_t osr_t) {
    uint8_t reg_data;
    if (!dev)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    reg_data = reg_read(dev, BMP5_REG_OSR_CONFIG);
    reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_TEMP_OS, (uint8_t)osr_t);
    reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_PRESS_OS, (uint8_t)osr_p);
    /* press_en 保持不变 */
    reg_write(dev, BMP5_REG_OSR_CONFIG, reg_data);

    dev->osr_p = osr_p;
    dev->osr_t = osr_t;
    return BMP580_OK;
}

bmp580_err_t BMP580_EnablePressure(bmp580_dev_t* dev, uint8_t enable) {
    if (!dev)
        return BMP580_ERR_PARAM;

    reg_rmw(dev, BMP5_REG_OSR_CONFIG, BMP5_PRESS_EN_MSK,
            (enable ? 0x01 : 0x00) << BMP5_PRESS_EN_POS);
    return BMP580_OK;
}

bmp580_err_t BMP580_ConfigODR(bmp580_dev_t* dev, bmp580_odr_t odr) {
    if (!dev)
        return BMP580_ERR_PARAM;
    reg_rmw(dev, BMP5_REG_ODR_CONFIG, BMP5_ODR_MSK, (uint8_t)odr << BMP5_ODR_POS);
    return BMP580_OK;
}

bmp580_err_t BMP580_ConfigMeasurement(bmp580_dev_t* dev,
                                      bmp580_osr_t osr_p,
                                      bmp580_osr_t osr_t,
                                      bmp580_odr_t odr,
                                      uint8_t press_enable) {
    uint8_t reg_data[2];

    if (!dev)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    /* Burst read: OSR_CONFIG + ODR_CONFIG */
    reg_read_burst(dev, BMP5_REG_OSR_CONFIG, reg_data, 2);

    /* OSR_CONFIG */
    reg_data[0] = BMP5_SET_BITS_POS_0(reg_data[0], BMP5_TEMP_OS, (uint8_t)osr_t);
    reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_PRESS_OS, (uint8_t)osr_p);
    reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_PRESS_EN, press_enable);

    /* ODR_CONFIG */
    reg_data[1] = BMP5_SET_BITSLICE(reg_data[1], BMP5_ODR, (uint8_t)odr);

    /* Burst write */
    reg_write_burst(dev, BMP5_REG_OSR_CONFIG, reg_data, 2);

    dev->osr_p = osr_p;
    dev->osr_t = osr_t;
    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  IIR 滤波器
 *  DSP_CONFIG(0x30): [6]=fifo_iir_p [5]=fifo_iir_t
 *                    [3]=shdw_iir_p [1]=shdw_iir_t
 *                    [0]=iir_flush_forced_en
 *  DSP_IIR(0x31):    [6:4]=set_iir_p [2:0]=set_iir_t
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigIIR(bmp580_dev_t* dev,
                              bmp580_iir_t iir_p,
                              bmp580_iir_t iir_t,
                              uint8_t flush_en) {
    uint8_t reg_data[2];

    if (!dev)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    /* Burst read: DSP_CONFIG + DSP_IIR */
    reg_read_burst(dev, BMP5_REG_DSP_CONFIG, reg_data, 2);

    /* DSP_CONFIG */
    reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_IIR_FLUSH_FORCED_EN, flush_en);
    /* 开启 shadow 以输出 IIR 滤波后的数据 */
    reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_SHDW_SET_IIR_TEMP,
                                    (iir_t != BMP580_IIR_BYPASS) ? 1 : 0);
    reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_SHDW_SET_IIR_PRESS,
                                    (iir_p != BMP580_IIR_BYPASS) ? 1 : 0);

    /* DSP_IIR: set_iir_t 在 [2:0], set_iir_p 在 [6:4] */
    reg_data[1] = (uint8_t)((uint8_t)iir_t & 0x07);
    reg_data[1] = BMP5_SET_BITSLICE(reg_data[1], BMP5_SET_IIR_PRESS, (uint8_t)iir_p);

    /* Burst write */
    reg_write_burst(dev, BMP5_REG_DSP_CONFIG, reg_data, 2);

    return BMP580_OK;
}

bmp580_err_t BMP580_SetDataSrc(bmp580_dev_t* dev,
                               uint8_t src_t,
                               uint8_t src_p) {
    if (!dev)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    uint8_t reg_data = reg_read(dev, BMP5_REG_DSP_CONFIG);
    reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_SET_FIFO_IIR_TEMP, src_t & 1);
    reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_SET_FIFO_IIR_PRESS, src_p & 1);
    reg_write(dev, BMP5_REG_DSP_CONFIG, reg_data);

    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  数据读取
 *
 *  TEMP_DATA_XLSB(0x1D) 连续读 6 字节:
 *    [0]=T_XLSB [1]=T_LSB [2]=T_MSB
 *    [3]=P_XLSB [4]=P_LSB [5]=P_MSB
 *
 *  温度: signed 24-bit / 65536  → °C
 *  压力: unsigned 24-bit / 64   → Pa
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ReadData(bmp580_dev_t* dev, bmp580_data_t* data) {
    uint8_t buf[6];
    int32_t raw_temp;
    uint32_t raw_press;

    if (!dev || !data)
        return BMP580_ERR_PARAM;

    reg_read_burst(dev, BMP5_REG_TEMP_DATA_XLSB, buf, 6);

    /* 温度: signed 24-bit，符号扩展 */
    raw_temp = (int32_t)((uint32_t)(((uint32_t)buf[2] << 16) |
                                    ((uint16_t)buf[1] << 8) |
                                    buf[0])
                         << 8) >>
               8;

    /* 压力: unsigned 24-bit */
    raw_press = (uint32_t)((uint32_t)(buf[5] << 16) |
                           (uint16_t)(buf[4] << 8) |
                           buf[3]);

    data->raw_temp = raw_temp;
    data->raw_press = (int32_t)raw_press;
    data->temperature_deg = (float)(raw_temp / 65536.0);
    data->pressure_pa = (float)(raw_press / 64.0);

    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  FORCED 模式
 * ═══════════════════════════════════════════════════ */

static const uint16_t osr_time_ms[8] = {
    2, 3, 4, 7, 12, 24, 45, 90};

bmp580_err_t BMP580_ForcedMeasure(bmp580_dev_t* dev, bmp580_data_t* data) {
    bmp580_mode_t mode;
    uint32_t elapsed = 0;
    uint32_t timeout;

    if (!dev || !data)
        return BMP580_ERR_PARAM;

    timeout = (uint32_t)osr_time_ms[dev->osr_p & 7] +
              (uint32_t)osr_time_ms[dev->osr_t & 7] + 10;

    /* 触发 Forced 模式（单次测量后自动回 Standby） */
    set_power_mode_raw(dev, BMP580_MODE_FORCED);

    /* 等待测量完成 */
    do {
        delay_ms(1);
        elapsed++;
        BMP580_GetPowerMode(dev, &mode);
        if (mode == BMP580_MODE_STANDBY)
            break;
    } while (elapsed < timeout);

    if (elapsed >= timeout)
        return BMP580_ERR_TIMEOUT;

    return BMP580_ReadData(dev, data);
}

/* ═══════════════════════════════════════════════════
 *  FIFO
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigFIFO(bmp580_dev_t* dev,
                               bmp580_fifo_frame_t frame_sel,
                               bmp580_fifo_mode_t mode,
                               uint8_t dec_sel,
                               uint8_t threshold) {
    uint8_t reg_data;

    if (!dev)
        return BMP580_ERR_PARAM;
    if (threshold > 31 || dec_sel > 7)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    /* FIFO_CONFIG */
    reg_data = reg_read(dev, BMP5_REG_FIFO_CONFIG);
    reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_FIFO_MODE, (uint8_t)mode);
    reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_FIFO_THRESHOLD, threshold);
    reg_write(dev, BMP5_REG_FIFO_CONFIG, reg_data);

    /* FIFO_SEL */
    reg_data = reg_read(dev, BMP5_REG_FIFO_SEL);
    reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_FIFO_FRAME_SEL, (uint8_t)frame_sel);
    reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_FIFO_DEC_SEL, dec_sel);
    reg_write(dev, BMP5_REG_FIFO_SEL, reg_data);

    return BMP580_OK;
}

bmp580_err_t BMP580_GetFIFOCount(bmp580_dev_t* dev, uint8_t* count) {
    if (!dev || !count)
        return BMP580_ERR_PARAM;

    *count = BMP5_GET_BITS_POS_0(reg_read(dev, BMP5_REG_FIFO_COUNT), BMP5_FIFO_COUNT);
    return BMP580_OK;
}

bmp580_err_t BMP580_ReadFIFO(bmp580_dev_t* dev,
                             uint8_t* buf,
                             uint32_t byte_count) {
    if (!dev || !buf || !byte_count)
        return BMP580_ERR_PARAM;

    reg_read_burst(dev, BMP5_REG_FIFO_DATA, buf, (uint16_t)byte_count);
    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  中断
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigInt(bmp580_dev_t* dev,
                              uint8_t src_bitmap,
                              uint8_t active_high,
                              uint8_t open_drain,
                              uint8_t latched) {
    uint8_t reg_data;
    uint8_t int_status;

    if (!dev)
        return BMP580_ERR_PARAM;

    /* 关闭所有中断源 */
    reg_write(dev, BMP5_REG_INT_SOURCE, 0x00);

    /* 清除中断状态 */
    int_status = reg_read(dev, BMP5_REG_INT_STATUS);

    /* 配置中断 */
    reg_data = reg_read(dev, BMP5_REG_INT_CONFIG);
    reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_INT_MODE, latched);
    reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_POL, active_high);
    reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_OD, open_drain);
    reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_EN, 1);
    reg_write(dev, BMP5_REG_INT_CONFIG, reg_data);

    /* 设置中断源 */
    reg_write(dev, BMP5_REG_INT_SOURCE, src_bitmap & 0x0F);

    return BMP580_OK;
}

bmp580_err_t BMP580_GetIntStatus(bmp580_dev_t* dev, uint8_t* status) {
    if (!dev || !status)
        return BMP580_ERR_PARAM;

    *status = reg_read(dev, BMP5_REG_INT_STATUS);
    return BMP580_OK;
}

/* ═══════════════════════════════════════════════════
 *  NVM
 * ═══════════════════════════════════════════════════ */

static bmp580_err_t nvm_wait_ready(const bmp580_dev_t* dev, uint32_t timeout_ms) {
    uint32_t t = 0;
    do {
        uint8_t status = reg_read(dev, BMP5_REG_STATUS);
        if (status & BMP5_INT_NVM_RDY)
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
    if (address < BMP5_NVM_START_ADDR || address > BMP5_NVM_END_ADDR)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    r = nvm_wait_ready(dev, 100);
    if (r)
        return r;

    /* 写地址，prog_en = 0 */
    reg_write(dev, BMP5_REG_NVM_ADDR, address & 0x3F);

    /* NVM 读序列 */
    reg_write(dev, BMP5_REG_CMD, BMP5_NVM_FIRST_CMND);
    reg_write(dev, BMP5_REG_CMD, BMP5_NVM_READ_ENABLE_CMND);
    delay_ms(1);

    r = nvm_wait_ready(dev, 100);
    if (r)
        return r;

    /* 验证 NVM 状态 */
    uint8_t status = reg_read(dev, BMP5_REG_STATUS);
    if (!(status & BMP5_INT_NVM_RDY) ||
        (status & BMP5_INT_NVM_ERR) ||
        (status & BMP5_INT_NVM_CMD_ERR))
        return BMP580_ERR_NVM;

    lsb = reg_read(dev, BMP5_REG_NVM_DATA_LSB);
    msb = reg_read(dev, BMP5_REG_NVM_DATA_MSB);
    *data = (uint16_t)((msb << 8) | lsb);

    return BMP580_OK;
}

bmp580_err_t BMP580_NVMWrite(bmp580_dev_t* dev,
                             uint8_t address,
                             uint16_t data) {
    bmp580_err_t r;
    uint8_t status;

    if (!dev)
        return BMP580_ERR_PARAM;
    if (address < BMP5_NVM_START_ADDR || address > BMP5_NVM_END_ADDR)
        return BMP580_ERR_PARAM;

    BMP580_SetPowerMode(dev, BMP580_MODE_STANDBY);

    r = nvm_wait_ready(dev, 100);
    if (r)
        return r;

    /* 写地址 + prog_en = 1 */
    reg_write(dev, BMP5_REG_NVM_ADDR, (address & 0x3F) | 0x80);
    reg_write(dev, BMP5_REG_NVM_DATA_LSB, (uint8_t)(data & 0xFF));
    reg_write(dev, BMP5_REG_NVM_DATA_MSB, (uint8_t)(data >> 8));

    /* NVM 写序列 */
    reg_write(dev, BMP5_REG_CMD, BMP5_NVM_FIRST_CMND);
    reg_write(dev, BMP5_REG_CMD, BMP5_NVM_WRITE_ENABLE_CMND);

    r = nvm_wait_ready(dev, 50);
    if (r)
        return r;

    /* 验证 NVM 状态 */
    status = reg_read(dev, BMP5_REG_STATUS);
    if (!(status & BMP5_INT_NVM_RDY) ||
        (status & BMP5_INT_NVM_ERR) ||
        (status & BMP5_INT_NVM_CMD_ERR))
        return BMP580_ERR_NVM;

    /* 清除 prog_en */
    reg_write(dev, BMP5_REG_NVM_ADDR, address & 0x3F);

    return BMP580_OK;
}

bmp580_err_t BMP580_ReadUID(bmp580_dev_t* dev, uint64_t* uid) {
    bmp580_err_t r;
    uint16_t nvm[4];

    if (!dev || !uid)
        return BMP580_ERR_PARAM;

    for (int i = 0; i < 4; i++) {
        r = BMP580_NVMRead(dev, 0x23 + (uint8_t)i, &nvm[i]);
        if (r)
            return r;
    }

    *uid = (((uint64_t)(nvm[3] & 0x00FF)) << 40) |
           (((uint64_t)nvm[2]) << 24) |
           (((uint64_t)nvm[1]) << 8) |
           (((uint64_t)(nvm[0] & 0xFF00)) >> 8);

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
#define ISA_BARO_EXP (ISA_R * ISA_L / (ISA_G * ISA_M))
#define ISA_SLP_EXP (-ISA_G * ISA_M / (ISA_R * ISA_L))

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
    return (t0k / ISA_L) * (1.0f - powf(press_pa / sea_level_pa, ISA_BARO_EXP));
}

float BMP580_AltitudeToSeaLevelPressure(float press_pa,
                                        float temperature_c,
                                        float altitude_m) {
    float t0k = temperature_c + 273.15f;
    return press_pa * powf(1.0f - ISA_L * altitude_m / t0k, ISA_SLP_EXP);
}
