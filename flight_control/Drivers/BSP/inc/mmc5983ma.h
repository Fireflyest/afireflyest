#ifndef __MMC5983MA_H
#define __MMC5983MA_H

#include <stddef.h>
#include <stdint.h>
#include "spi_sensor.h"

/* ═══════════════════════════════════════════════════
 *  寄存器地址
 * ═══════════════════════════════════════════════════ */

#define MMC_REG_XOUT0 0x00      /* Xout[17:10] */
#define MMC_REG_XOUT1 0x01      /* Xout[9:2]   */
#define MMC_REG_YOUT0 0x02      /* Yout[17:10] */
#define MMC_REG_YOUT1 0x03      /* Yout[9:2]   */
#define MMC_REG_ZOUT0 0x04      /* Zout[17:10] */
#define MMC_REG_ZOUT1 0x05      /* Zout[9:2]   */
#define MMC_REG_XYZOUT2 0x06    /* Xout[1:0], Yout[1:0], Zout[1:0] */
#define MMC_REG_TOUT 0x07       /* Temperature output */
#define MMC_REG_STATUS 0x08     /* Device status */
#define MMC_REG_CTRL0 0x09      /* Internal Control 0 */
#define MMC_REG_CTRL1 0x0A      /* Internal Control 1 */
#define MMC_REG_CTRL2 0x0B      /* Internal Control 2 */
#define MMC_REG_CTRL3 0x0C      /* Internal Control 3 */
#define MMC_REG_PRODUCT_ID 0x2F /* Product ID */

/* ═══════════════════════════════════════════════════
 *  常量
 * ═══════════════════════════════════════════════════ */

#define MMC_PRODUCT_ID 0x30 /* 预期 Product ID */

/* STATUS 寄存器位 */
#define MMC_STATUS_MEAS_M_DONE (1u << 0)
#define MMC_STATUS_MEAS_T_DONE (1u << 1)
#define MMC_STATUS_OTP_RD_DONE (1u << 4)

/* CTRL0 位 */
#define MMC_CTRL0_TM_M (1u << 0)        /* 触发磁场测量 */
#define MMC_CTRL0_TM_T (1u << 1)        /* 触发温度测量 */
#define MMC_CTRL0_INT_MEAS_EN (1u << 2) /* 测量完成中断使能 */
#define MMC_CTRL0_SET (1u << 3)         /* SET 操作 */
#define MMC_CTRL0_RESET (1u << 4)       /* RESET 操作 */
#define MMC_CTRL0_AUTO_SR_EN (1u << 5)  /* 自动 SET/RESET */
#define MMC_CTRL0_OTP_READ (1u << 6)    /* 重新读 OTP */

/* CTRL1 位 */
#define MMC_CTRL1_BW0 (1u << 0)
#define MMC_CTRL1_BW1 (1u << 1)
#define MMC_CTRL1_X_INHIBIT (1u << 2)
#define MMC_CTRL1_YZ_INHIBIT_L (1u << 3) /* bit3 + bit2 = YZ-inhibit */
#define MMC_CTRL1_SW_RST (1u << 7)

/* CTRL2 位 */
#define MMC_CTRL2_CMM_EN (1u << 0)     /* 连续测量使能 */
#define MMC_CTRL2_CM_FREQ_MASK (0x07u) /* CM_Freq[2:0] 低 3 位 */
#define MMC_CTRL2_PRD_SET_SHIFT 4
#define MMC_CTRL2_PRD_SET_MASK (0x07u << 4)
#define MMC_CTRL2_EN_PRD_SET (1u << 7) /* 周期 SET 使能 */

/* CTRL3 位 */
#define MMC_CTRL3_ST_ENP (1u << 1) /* 正向电流自检 */
#define MMC_CTRL3_ST_ENM (1u << 2) /* 反向电流自检 */
#define MMC_CTRL3_SPI_3W (1u << 6) /* 3-wire SPI 模式 */

/* 空场输出 (Null Field Output) */
#define MMC_NULL_FIELD_16BIT 32768
#define MMC_NULL_FIELD_18BIT 131072

/* ═══════════════════════════════════════════════════
 *  枚举：带宽 / 测量时间
 *
 *  BW[1:0] | 测量时间 | 带宽   | 典型 RMS 噪声 | 最大 ODR
 *  --------+---------+-------+--------------+--------
 *    00    |  8 ms    | 100Hz |  0.4 mG      |   50 Hz
 *    01    |  4 ms    | 200Hz |  0.6 mG      |  100 Hz
 *    10    |  2 ms    | 400Hz |  0.8 mG      |  225 Hz
 *    11    | 0.5 ms   | 800Hz |  1.2 mG      |  580 Hz
 * ═══════════════════════════════════════════════════ */

typedef enum {
    MMC_BW_00 = 0x00, /* 8ms,   100Hz, 0.4mG 噪声, 最大 50Hz ODR  */
    MMC_BW_01 = 0x01, /* 4ms,   200Hz, 0.6mG 噪声, 最大 100Hz ODR */
    MMC_BW_10 = 0x02, /* 2ms,   400Hz, 0.8mG 噪声, 最大 225Hz ODR */
    MMC_BW_11 = 0x03, /* 0.5ms, 800Hz, 1.2mG 噪声, 最大 580Hz ODR */
} mmc5983ma_bw_t;

/* ═══════════════════════════════════════════════════
 *  枚举：连续测量频率
 *
 *  CM_Freq[2:0] | 典型频率
 *  -------------+--------
 *      000      | 关闭
 *      001      |  1 Hz
 *      010      | 10 Hz
 *      011      | 20 Hz
 *      100      | 50 Hz
 *      101      | 100 Hz
 *      110      | 200 Hz (需 BW=01)
 *      111      | 1000 Hz (需 BW=11)
 * ═══════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════
 *  枚举：周期性 SET 间隔（测量次数）
 * ═══════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════
 *  枚举：错误码
 * ═══════════════════════════════════════════════════ */

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

/** @brief 原始 XYZ 数据 (18-bit 无符号) */
typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} mmc5983ma_raw_data_t;

/** @brief 换算后数据 (Gauss) */
typedef struct {
    float x; /* X 轴, Gauss */
    float y; /* Y 轴, Gauss */
    float z; /* Z 轴, Gauss */
} mmc5983ma_data_t;

/** @brief 设备句柄 */
typedef struct {
    uint8_t sensor_id;
    mmc5983ma_bw_t bw; /* 当前带宽设置 */
    uint8_t use_18bit; /* 0=16bit, 1=18bit */
} mmc5983ma_dev_t;

/* ═══════════════════════════════════════════════════
 *  延时
 * ═══════════════════════════════════════════════════ */

typedef void (*mmc5983ma_delay_fn_t)(uint32_t ms);
void MMC5983MA_SetDelay(mmc5983ma_delay_fn_t fn);

/* ═══════════════════════════════════════════════════
 *  基础 API
 * ═══════════════════════════════════════════════════ */

/**
 * @brief  读取 Product ID（预期 0x30）
 */
mmc5983ma_err_t MMC5983MA_WhoAmI(mmc5983ma_dev_t* dev, uint8_t* id);

/**
 * @brief  初始化（软复位 + 验证 ID + 读 OTP）
 */
mmc5983ma_err_t MMC5983MA_Init(mmc5983ma_dev_t* dev);

/**
 * @brief  软复位
 */
mmc5983ma_err_t MMC5983MA_SoftReset(mmc5983ma_dev_t* dev);

/* ═══════════════════════════════════════════════════
 *  配置 API
 * ═══════════════════════════════════════════════════ */

/**
 * @brief  设置带宽（影响测量时间、噪声、最大 ODR）
 */
mmc5983ma_err_t MMC5983MA_SetBandwidth(mmc5983ma_dev_t* dev,
                                       mmc5983ma_bw_t bw);

/**
 * @brief  使能/禁止测量完成中断
 */
mmc5983ma_err_t MMC5983MA_EnableIntMeasDone(mmc5983ma_dev_t* dev,
                                            uint8_t enable);

/**
 * @brief  使能/禁止自动 SET/RESET
 */
mmc5983ma_err_t MMC5983MA_EnableAutoSR(mmc5983ma_dev_t* dev,
                                       uint8_t enable);

/**
 * @brief  使能连续测量模式
 * @param  freq    测量频率
 * @param  prd_set 周期 SET 间隔（每 N 次测量执行一次 SET）
 */
mmc5983ma_err_t MMC5983MA_EnableContinuousMode(mmc5983ma_dev_t* dev,
                                               mmc5983ma_cm_freq_t freq,
                                               mmc5983ma_prd_set_t prd_set);

/**
 * @brief  禁止连续测量模式
 */
mmc5983ma_err_t MMC5983MA_DisableContinuousMode(mmc5983ma_dev_t* dev);

/* ═══════════════════════════════════════════════════
 *  SET / RESET
 * ═══════════════════════════════════════════════════ */

/**
 * @brief  执行 SET 操作（500ns 脉冲，自动清除）
 */
mmc5983ma_err_t MMC5983MA_Set(mmc5983ma_dev_t* dev);

/**
 * @brief  执行 RESET 操作（500ns 脉冲，自动清除）
 */
mmc5983ma_err_t MMC5983MA_Reset(mmc5983ma_dev_t* dev);

/* ═══════════════════════════════════════════════════
 *  单次测量 API
 * ═══════════════════════════════════════════════════ */

/**
 * @brief  触发一次磁场测量 → 等待完成 → 读取 XYZ
 *
 *  raw->x/y/z 为 18-bit 无符号值。
 *  16-bit 模式下 Null field = 32768。
 *  18-bit 模式下 Null field = 131072。
 */
mmc5983ma_err_t MMC5983MA_ReadMagRaw(mmc5983ma_dev_t* dev,
                                     mmc5983ma_raw_data_t* raw);

/**
 * @brief  触发一次磁场测量 → 换算为 Gauss
 */
mmc5983ma_err_t MMC5983MA_ReadMag(mmc5983ma_dev_t* dev,
                                  mmc5983ma_data_t* data);

/**
 * @brief  触发一次温度测量 → 等待完成 → 读取温度
 *
 *  temp_out 范围 0~255，0 对应 -75°C，每 LSB ≈ 0.8°C
 */
mmc5983ma_err_t MMC5983MA_ReadTemp(mmc5983ma_dev_t* dev,
                                   float* temperature_c);

/* ═══════════════════════════════════════════════════
 *  连续模式读取 API（配合 Cmm_en）
 * ═══════════════════════════════════════════════════ */

/**
 * @brief  检查磁场数据是否就绪（轮询 STATUS.Meas_M_Done）
 */
mmc5983ma_err_t MMC5983MA_IsMagDataReady(mmc5983ma_dev_t* dev,
                                         uint8_t* ready);

/**
 * @brief  仅读取已就绪的磁场数据（不触发新测量）
 *
 *  适用于连续测量模式下，轮询 ready 后调用此函数。
 */
mmc5983ma_err_t MMC5983MA_ReadMagRawOnly(mmc5983ma_dev_t* dev,
                                         mmc5983ma_raw_data_t* raw);

/**
 * @brief  仅读取已就绪的磁场数据并换算为 Gauss
 */
mmc5983ma_err_t MMC5983MA_ReadMagOnly(mmc5983ma_dev_t* dev,
                                      mmc5983ma_data_t* data);

/* ═══════════════════════════════════════════════════
 *  航向角计算
 * ═══════════════════════════════════════════════════ */

/**
 * @brief  计算磁航向角（假设传感器水平放置）
 *
 *  heading = atan2(Y, X)，结果 0~360°。
 *  仅在 XY 平面水平时准确；倾斜时需要加速度计补偿。
 *
 *  @param  mag      Gauss 数据
 *  @return 航向角 (°)，北=0，东=90
 */
float MMC5983MA_CalcHeading(const mmc5983ma_data_t* mag);

/* ═══════════════════════════════════════════════════
 *  SET/RESET 消除偏移的高精度测量
 * ═══════════════════════════════════════════════════ */

/**
 * @brief  使用 SET + 测量 + RESET + 测量 方法消除电桥偏移
 *
 *  H = (Output_SET - Output_RESET) / 2
 *  Offset = (Output_SET + Output_RESET) / 2
 *
 *  可消除温度引起的偏移漂移，提升航向精度。
 *
 *  @param  dev      设备
 *  @param  data     输出: 消除偏移后的 Gauss 值
 *  @param  offset   可选输出: 偏移量 (Gauss)，不需要可传 NULL
 */
mmc5983ma_err_t MMC5983MA_ReadMagCompensated(mmc5983ma_dev_t* dev,
                                             mmc5983ma_data_t* data,
                                             mmc5983ma_data_t* offset);

#endif /* __MMC5983MA_H */
