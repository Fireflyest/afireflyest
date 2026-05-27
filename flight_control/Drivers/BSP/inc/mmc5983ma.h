#ifndef __MMC5983MA_H
#define __MMC5983MA_H

#include <stddef.h>
#include <stdint.h>
#include "spi_sensor.h"

/* ═══════════════════════════════════════════════════
 *  寄存器地址
 * ═══════════════════════════════════════════════════ */

#define MMC_REG_XOUT0 0x00
#define MMC_REG_XOUT1 0x01
#define MMC_REG_YOUT0 0x02
#define MMC_REG_YOUT1 0x03
#define MMC_REG_ZOUT0 0x04
#define MMC_REG_ZOUT1 0x05
#define MMC_REG_XYZOUT2 0x06
#define MMC_REG_TOUT 0x07
#define MMC_REG_STATUS 0x08
#define MMC_REG_CTRL0 0x09
#define MMC_REG_CTRL1 0x0A
#define MMC_REG_CTRL2 0x0B
#define MMC_REG_CTRL3 0x0C
#define MMC_REG_PRODUCT_ID 0x2F
#define MMC_PRODUCT_ID 0x30

/* STATUS 位 */
#define MMC_STATUS_MEAS_M_DONE (1u << 0)
#define MMC_STATUS_MEAS_T_DONE (1u << 1)
#define MMC_STATUS_OTP_RD_DONE (1u << 4)

/* CTRL0 位 */
#define MMC_CTRL0_TM_M (1u << 0)
#define MMC_CTRL0_TM_T (1u << 1)
#define MMC_CTRL0_INT_MEAS_EN (1u << 2)
#define MMC_CTRL0_SET (1u << 3)
#define MMC_CTRL0_RESET (1u << 4)
#define MMC_CTRL0_AUTO_SR_EN (1u << 5)
#define MMC_CTRL0_OTP_READ (1u << 6)

/* CTRL1 位 */
#define MMC_CTRL1_BW0 (1u << 0)
#define MMC_CTRL1_BW1 (1u << 1)
#define MMC_CTRL1_X_INHIBIT (1u << 3)
#define MMC_CTRL1_YZ_INHIBIT_L (1u << 4)
#define MMC_CTRL1_SW_RST (1u << 7)

/* CTRL2 位 */
#define MMC_CTRL2_CMM_EN (1u << 3)
#define MMC_CTRL2_EN_PRD_SET (1u << 7)

/* 空场输出 */
#define MMC_NULL_FIELD_16BIT 32768
#define MMC_NULL_FIELD_18BIT 131072

/* ═══════════════════════════════════════════════════
 *  枚举
 * ═══════════════════════════════════════════════════ */

typedef enum {
    MMC_BW_00 = 0x00,
    MMC_BW_01 = 0x01,
    MMC_BW_10 = 0x02,
    MMC_BW_11 = 0x03,
} mmc5983ma_bw_t;

typedef enum {
    MMC_CM_FREQ_OFF = 0x00,
    MMC_CM_FREQ_1HZ = 0x01,
    MMC_CM_FREQ_10HZ = 0x02,
    MMC_CM_FREQ_20HZ = 0x03,
    MMC_CM_FREQ_50HZ = 0x04,
    MMC_CM_FREQ_100HZ = 0x05,
    MMC_CM_FREQ_200HZ = 0x06,
    MMC_CM_FREQ_1000HZ = 0x07,
} mmc5983ma_cm_freq_t;

typedef enum {
    MMC_PRD_SET_1 = 0x00,
    MMC_PRD_SET_25 = 0x01,
    MMC_PRD_SET_75 = 0x02,
    MMC_PRD_SET_100 = 0x03,
    MMC_PRD_SET_250 = 0x04,
    MMC_PRD_SET_500 = 0x05,
    MMC_PRD_SET_1000 = 0x06,
    MMC_PRD_SET_2000 = 0x07,
} mmc5983ma_prd_set_t;

typedef enum {
    MMC_OK = 0,
    MMC_ERR_PARAM = -1,
    MMC_ERR_PRODUCT_ID = -2,
    MMC_ERR_COMM = -3,
    MMC_ERR_TIMEOUT = -4,
    MMC_ERR_OTP = -5,
} mmc5983ma_err_t;

/* ═══════════════════════════════════════════════════
 *  数据结构
 * ═══════════════════════════════════════════════════ */

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} mmc5983ma_raw_data_t;

typedef struct {
    float x;
    float y;
    float z;
} mmc5983ma_data_t;

typedef struct {
    uint8_t sensor_id;
    mmc5983ma_bw_t bw;
    uint8_t use_18bit;
} mmc5983ma_dev_t;

/* ═══════════════════════════════════════════════════
 *  函数声明
 * ═══════════════════════════════════════════════════ */

typedef void (*mmc5983ma_delay_fn_t)(uint32_t ms);
void MMC5983MA_SetDelay(mmc5983ma_delay_fn_t fn);

mmc5983ma_err_t MMC5983MA_WhoAmI(mmc5983ma_dev_t* dev, uint8_t* id);
mmc5983ma_err_t MMC5983MA_Init(mmc5983ma_dev_t* dev);
mmc5983ma_err_t MMC5983MA_SoftReset(mmc5983ma_dev_t* dev);

mmc5983ma_err_t MMC5983MA_SetBandwidth(mmc5983ma_dev_t* dev, mmc5983ma_bw_t bw);
mmc5983ma_err_t MMC5983MA_Set(mmc5983ma_dev_t* dev);
mmc5983ma_err_t MMC5983MA_Reset(mmc5983ma_dev_t* dev);

mmc5983ma_err_t MMC5983MA_ReadMagRaw(mmc5983ma_dev_t* dev, mmc5983ma_raw_data_t* raw);
mmc5983ma_err_t MMC5983MA_ReadMag(mmc5983ma_dev_t* dev, mmc5983ma_data_t* data);
mmc5983ma_err_t MMC5983MA_ReadTemp(mmc5983ma_dev_t* dev, float* temperature_c);

mmc5983ma_err_t MMC5983MA_IsMagDataReady(mmc5983ma_dev_t* dev, uint8_t* ready);
mmc5983ma_err_t MMC5983MA_ReadMagRawOnly(mmc5983ma_dev_t* dev, mmc5983ma_raw_data_t* raw);

float MMC5983MA_CalcHeading(const mmc5983ma_data_t* mag);

mmc5983ma_err_t MMC5983MA_ReadMagCompensated(mmc5983ma_dev_t* dev,
                                             mmc5983ma_data_t* data,
                                             mmc5983ma_data_t* offset);

#endif
