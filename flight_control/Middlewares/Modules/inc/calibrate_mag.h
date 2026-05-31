#ifndef __CALIBRATE_MAG_H
#define __CALIBRATE_MAG_H

#include "stm32f4xx.h"

/**
 * @brief 磁力计校准 — 硬铁偏置 + 简单软铁比例因子
 *
 * 原理:
 *   理想情况下，水平旋转一圈的磁力计读数构成一个球面:
 *     (mx - bx)² + (my - by)² + (mz - bz)² = R²
 *   其中 [bx, by, bz] 就是硬铁偏置 (球心)。
 *
 *   球面拟合:
 *     bias[i]  = (max[i] + min[i]) / 2
 *     radius[i] = (max[i] - min[i]) / 2
 *     scale[i]  = avg_radius / radius[i]   (简单软铁补偿)
 *
 * 校准流程:
 *   1. 调用 Calib_Mag_Start() 开始采集
 *   2. 缓慢旋转飞行器，尽量覆盖所有方向 (八字形旋转最佳)
 *   3. 内部自动检测各轴覆盖度，满足条件后自动计算
 *   4. 调用 Calib_Mag_Apply() 应用校准结果
 */

/* 采集参数 */
#define CALIB_MAG_MIN_SAMPLES 500  /**< 最少样本数               */
#define CALIB_MAG_MAX_SAMPLES 5000 /**< 最多样本数 (超时)         */
#define CALIB_MAG_MIN_RANGE 20.0f  /**< 各轴最小变化范围 (μT)     */

/**
 * @brief 磁力计校准参数
 */
typedef struct {
    float bias[3];     /**< 硬铁偏置 (μT)                   */
    float scale[3];    /**< 软铁比例因子 (无量纲, 理想为1)    */
    float total_field; /**< 校准后总场强度 (μT, 用于健康检查) */
    uint8_t is_valid;  /**< 校准是否有效                     */
} Calib_Mag_t;

/**
 * @brief 校准状态
 */
typedef enum {
    CALIB_MAG_IDLE = 0,   /**< 空闲                        */
    CALIB_MAG_COLLECTING, /**< 正在采集 (需要用户旋转飞行器) */
    CALIB_MAG_DONE,       /**< 校准完成                     */
    CALIB_MAG_FAILED      /**< 校准失败 (覆盖度不足)         */
} Calib_Mag_State_t;

/**
 * @brief 校准句柄
 */
typedef struct {
    Calib_Mag_t calib;
    Calib_Mag_State_t state;

    float mag_min[3];      /**< 采集期间各轴最小值 (μT)      */
    float mag_max[3];      /**< 采集期间各轴最大值 (μT)      */
    uint32_t sample_count; /**< 已采集样本数                 */
    uint8_t coverage_ok;   /**< 各轴覆盖度是否达标           */
} Calib_Mag_Handle_t;

/* 接口函数 */

/**
 * @brief 初始化校准句柄
 */
void Calib_Mag_Init(Calib_Mag_Handle_t* handle);

/**
 * @brief 开始磁力计校准
 *        之后每次调用 Calib_Mag_AddSample() 送入新数据
 */
void Calib_Mag_Start(Calib_Mag_Handle_t* handle);

/**
 * @brief 送入一个磁力计样本
 * @param[in] handle 校准句柄
 * @param[in] mag    原始磁力计数据 [mx, my, mz] (μT)
 */
void Calib_Mag_AddSample(Calib_Mag_Handle_t* handle, const float mag[3]);

/**
 * @brief 获取当前校准进度 (0.0 ~ 1.0)
 * @return 各轴覆盖度的最小值; 1.0 = 已满足校准条件
 */
float Calib_Mag_GetProgress(Calib_Mag_Handle_t* handle);

/**
 * @brief 获取校准状态
 */
Calib_Mag_State_t Calib_Mag_GetState(Calib_Mag_Handle_t* handle);

/**
 * @brief 应用校准: corrected = (raw - bias) * scale
 */
void Calib_Mag_Apply(const Calib_Mag_t* calib,
                     const float raw[3],
                     float corrected[3]);

/**
 * @brief 加载已有的校准参数
 */
void Calib_Mag_Load(Calib_Mag_Handle_t* handle, const Calib_Mag_t* calib);

#endif /* __CALIBRATE_MAG_H */
