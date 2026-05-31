/**
 * @file ekf_core.h
 * @brief EKF 四旋翼无人机 — 核心算法接口
 *
 * Error-State EKF (ESEKF)，15 维误差状态:
 *   δx = [δp(3), δv(3), δθ(3), δb_g(3), δb_a(3)]
 *
 * 调用流程:
 *   1. ekf_init()                          初始化
 *   2. ekf_align(imu, mag)                 静止对准 (执行一次)
 *   3. 循环:
 *      - ekf_predict(imu)                  每收到 IMU 数据调用 (400Hz)
 *      - ekf_update_mag(mag)               磁力计更新 (100Hz)
 *      - ekf_update_gps(gps)               GPS 更新 (10Hz)
 *      - ekf_update_baro(baro)             气压计更新 (50Hz)
 *      - ekf_update_optflow(flow)          光流更新 (50Hz)
 */

#ifndef EKF_CORE_H
#define EKF_CORE_H

#include "ekf_sensors.h"
#include "ekf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*  EKF 实例结构体                                                             */
/* ========================================================================== */

/**
 * @brief EKF 主结构体
 *
 * 包含完整的 EKF 运行时状态:
 *   - state:     标称状态 (16 维)
 *   - P:         误差状态协方差 (15×15)
 *   - noise:     传感器噪声参数
 *   - mag_ref:   地磁参考向量
 *   - gps_origin:GPS 坐标原点
 *   - last_predict_us: 上次预测时间戳
 *   - initialized: 对准完成标志
 */
typedef struct {
    ekf_state_t state;
    ekf_cov_t P;
    ekf_noise_params_t noise;
    ekf_mag_reference_t mag_ref;
    ekf_gps_origin_t gps_origin;

    uint64_t last_predict_us;
    uint8_t initialized;
    int baro_altitude_initialized;
} ekf_t;

/* ========================================================================== */
/*  生命周期                                                                   */
/* ========================================================================== */

/**
 * @brief 初始化 EKF 实例
 * @param[out] ekf EKF 实例
 * @param[in]  noise 传感器噪声参数 (NULL 则使用默认值)
 */
void ekf_init(ekf_t* ekf, const ekf_noise_params_t* noise);

/**
 * @brief 设置 EKF 初始高度 (NED 坐标)
 * @param ekf       EKF 实例
 * @param altitude  初始高度 (向上为正, m)
 */
void ekf_set_init_altitude(ekf_t* ekf, float altitude);

/**
 * @brief 静止对准 — 初始化姿态和 bias
 *
 * 调用时机: 无人机水平静止放置后，采集若干 IMU 和磁力计数据后调用。
 * 内部使用:
 *   - 加速度计估计 roll/pitch
 *   - 磁力计估计 yaw (补偿磁偏角)
 *   - IMU 均值作为初始 bias 估计
 *
 * @param[in,out] ekf   EKF 实例
 * @param[in]     imu   静止期间的 IMU 数据数组
 * @param[in]     imu_n IMU 数据数量
 * @param[in]     mag   静止期间的磁力计数据 (NULL 则跳过 yaw 对准)
 * @param[in]     mag_n 磁力计数据数量
 * @return 0=成功, -1=数据不足, -2=加速度异常
 */
int ekf_align(ekf_t* ekf,
              const ekf_imu_t imu[],
              int imu_n,
              const ekf_mag_t mag[],
              int mag_n);

/* ========================================================================== */
/*  预测 (IMU 驱动)                                                            */
/* ========================================================================== */

/**
 * @brief 状态预测 — 每收到 IMU 数据调用
 *
 * 执行:
 *   1. 标称状态传播 (四元数精确积分, 中点法速度/位置)
 *   2. 误差状态协方差传播 (线性化 F, G, 离散化 Q)
 *
 * @param[in,out] ekf EKF 实例
 * @param[in]     imu 当前 IMU 数据 (陀螺仪 + 加速度计)
 */
void ekf_predict(ekf_t* ekf, const ekf_imu_t* imu);

/* ========================================================================== */
/*  量测更新 (异步调用)                                                         */
/* ========================================================================== */

/**
 * @brief 磁力计更新 — 修正 yaw
 * @param[in,out] ekf EKF 实例
 * @param[in]     mag 磁力计数据 (机体系 FRD, μT)
 */
void ekf_update_mag(ekf_t* ekf, const ekf_mag_t* mag);

/**
 * @brief GPS 更新 — 修正位置和速度
 *
 * 首次收到有效 GPS 数据时自动初始化坐标原点。
 *
 * @param[in,out] ekf EKF 实例
 * @param[in]     gps GPS 数据 (WGS-84)
 */
void ekf_update_gps(ekf_t* ekf, const ekf_gps_t* gps);

/**
 * @brief 气压计更新 — 修正高度
 * @param[in,out] ekf  EKF 实例
 * @param[in]     baro 气压计数据 (相对高度, m, 向上为正)
 */
void ekf_update_baro(ekf_t* ekf, const ekf_baro_t* baro);

/**
 * @brief 光流更新 — 修正水平速度
 * @param[in,out] ekf  EKF 实例
 * @param[in]     flow 光流数据
 */
void ekf_update_optflow(ekf_t* ekf, const ekf_optflow_t* flow);

/* ========================================================================== */
/*  状态读取                                                                   */
/* ========================================================================== */

/**
 * @brief 获取当前欧拉角 (rad)
 * @param[in]  ekf  EKF 实例
 * @param[out] out  [roll, pitch, yaw] (rad)
 */
void ekf_get_euler(const ekf_t* ekf, ekf_euler_t* out);

/**
 * @brief 获取 NED 位置 (m)
 */
void ekf_get_position(const ekf_t* ekf, ekf_vec3_t* out);

/**
 * @brief 获取 NED 速度 (m/s)
 */
void ekf_get_velocity(const ekf_t* ekf, ekf_vec3_t* out);

/**
 * @brief 获取四元数
 */
void ekf_get_quat(const ekf_t* ekf, ekf_quat_t* out);

/**
 * @brief 获取陀螺仪 bias 估计 (rad/s)
 */
void ekf_get_gyro_bias(const ekf_t* ekf, ekf_vec3_t* out);

/**
 * @brief 检查 EKF 是否已完成对准
 * @return 1=已对准, 0=未对准
 */
int ekf_is_initialized(const ekf_t* ekf);

#ifdef __cplusplus
}
#endif

#endif /* EKF_CORE_H */
