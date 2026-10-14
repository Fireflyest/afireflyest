/**
 * @file attitude.h
 * @brief 姿态估计模块公共接口
 *
 * 坐标系: NED (世界系) + FRD (机体系)
 * 四元数: Hamilton, 标量在前 [w, x, y, z]
 * 欧拉角: ZYX 顺序, Roll(X)/Pitch(Y)/Yaw(Z), 单位 rad
 *   - Roll:  右倾为正
 *   - Pitch: 抬头为正
 *   - Yaw:   机头右偏为正 (俯视顺时针)
 *
 * 加速度计: 水平静止输出 [0, 0, -g] (m/s²)
 * 陀螺仪:   rad/s, 正方向与欧拉角一致
 */

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>
#include "spatial_math.h"

extern uint8_t imu_rx_buf[14];
extern uint8_t mag_rx_buf[6];
extern float altitude_rx;
extern float temperature_rx;

/**
 * @brief 初始化姿态模块
 * @param accel_bias  加速度计偏置 (已有校准数据时传入, 否则传 {0,0,0})
 * @param accel_scale 加速度计缩放 (已有校准数据时传入, 否则传 {1,1,1})
 * @param init_altitude 初始高度 (m, 向上为正)
 */
void Attitude_Init(sm_vec3_t accel_bias, sm_vec3_t accel_scale, float init_altitude);

/**
 * @brief 周期性更新 (每次收到新 IMU 数据时调用)
 * @param dt 时间步长 (s)
 *
 * 内部流程:
 *   1. 解析原始 IMU/Mag 数据
 *   2. 加速度计校准补偿
 *   3. 前 200 帧: 静止对准 (估计初始姿态和 gyro bias)
 *   4. 之后: ekf_predict → ekf_update_mag → ekf_update_baro
 */
void Attitude_Update(float dt);

/**
 * @brief 判断当前是否静止 (用于加速度计六面校准)
 * @param[out] still 1=静止, 0=运动
 */
void Attitude_IsStill(uint8_t* still);

/** @brief 获取欧拉角 (rad) */
void Attitude_GetEuler(float* yaw, float* pitch, float* roll);

/** @brief 获取四元数 [w, x, y, z] */
void Attitude_GetQuat(sm_quat_t q);

/** @brief 获取 bias 补偿后的陀螺仪 (rad/s) */
void Attitude_GetGyro(sm_vec3_t gyro);

/** @brief 获取校准后的加速度 (m/s²) */
void Attitude_GetAccel(sm_vec3_t accel);

/** @brief 获取磁力计原始值 (μT) */
void Attitude_GetMag(sm_vec3_t mag);

/** @brief 获取融合高度 (m, 向上为正) */
void Attitude_GetAltitude(float* altitude);

/** @brief 获取垂直速度 (m/s, 向上为正) */
void Attitude_GetVelocityZ(float* velocityZ);

/** @brief 启动加速度计六面校准 */
void Attitude_Calibrate(void);

/** @brief 获取当前校准面编号 */
void Attitude_CalibratingFace(uint8_t* face);

/**
 * @brief 开始磁力计校准
 *        调用后缓慢旋转飞行器，覆盖所有方向 (约 10 秒)
 */
void Attitude_StartMagCalibrate(void);

/**
 * @brief 获取磁力计校准进度 (0.0 ~ 1.0)
 * @return 各轴覆盖度最小值, 1.0 = 校准条件已满足
 */
float Attitude_MagCalibrateProgress(void);

/**
 * @brief 获取磁力计校准状态
 * @return 0=空闲, 1=采集中, 2=完成, 3=失败
 */
uint8_t Attitude_MagCalibStatus(void);

uint8_t Attitude_MagCalibValid(void);

/**
 * @brief 姿态是否已完成对准 (EKF 就绪)
 * @note  磁力计校准采集期间对准被阻塞, 此时禁止解锁
 */
uint8_t Attitude_IsAligned(void);

/**
 * @brief 对准状态诊断
 * @param state   0=对准中(ATT_ALIGNING), 1=已对准(RUNNING)
 * @param cnt     IMU 好帧计数 (静止累积, 运动清零; ≥3 或兕底 200 才对准)
 * @param magCnt  磁力计好帧计数 (模长 20~100µT 才计; >10 才建参考)
 */
void Attitude_AlignStats(uint8_t* state, int* cnt, int* magCnt);

/**
 * @brief EKF 地磁参考是否已标定
 * @return 1 = 磁力计正在修正 yaw (ekf_update_mag 实际生效);
 *         0 = 未生效 (未对准/校准参数无效/场强异常被拒)
 */
uint8_t Attitude_MagRefCalibrated(void);

/**
 * @brief 磁力计更新统计 (诊断)
 * @param ok   累计应用次数 (可 NULL)
 * @param rej  累计被野值门控拒绝次数 (可 NULL)
 *            漂移且 rej 快速增长 → 修正被门控拒掉, 是门控/噪声参数问题
 */
void Attitude_MagStats(uint32_t* ok, uint32_t* rej);

/**
 * @brief 磁场新息模长 |实测 - R·参考| (µT)
 * @return 非负值; -1 = 地磁参考未标定
 *         漂移且此值持续偏大 → 参考/校准不一致;
 *         漂移且此值 ≈0 → 修正正常, 问题不在磁力计链路
 */
float Attitude_MagInnov(void);

/**
 * @brief 当前实测磁场模长 |mag_current| (µT, 校准后)
 * @return 稳定应 25~65; 随姿态大幅乱变 → 校准参数坏, 需重新校准
 */
float Attitude_MagFieldNorm(void);

/**
 * @brief 地磁参考模长 |m_earth| (µT)
 * @return 稳定应 25~65; -1 = 参考未标定
 */
float Attitude_MagRefNorm(void);

/**
 * @brief 磁场新息向量 (实测 - R·参考), 机体系 µT
 * @param v  输出 3 分量 (可 NULL)
 * @return 1 = 有效; 0 = 地磁参考未标定 (v 置 0)
 * @note   旋转测试: 转90° 某分量突变 → 该轴映射问题的线索
 */
int Attitude_MagInnovVec(float v[3]);

/**
 * @brief 预测的机体系磁场 hm = R · m_earth (µT)
 * @note  未标定时 v 置 0; 与实测 Attitude_GetMag 对比可定位错误轴
 */
void Attitude_MagPredicted(float v[3]);

/**
 * @brief 新息 EMA (τ=0.3s), 调试读数用
 * @note  静止时应稳定 (≈噪声均值); 旋转后 1s 内收敛到新值
 */
void Attitude_MagInnovEma(float v[3]);

/**
 * @brief 最近 1s 窗口内新息各轴峰峰值的最大值 (µT)
 * @return 非负; -1 = 参考未标定。静止时应小 (几 µT), 阶跃/故障时大
 */
float Attitude_MagInnovPp(void);

/**
 * @brief 连续化 yaw (°), 诊断显示用
 * @return 跨 ±180 wrap 不跳变的累计 yaw; 绕圈后可超出 ±180
 */
float Attitude_YawUnwrapped(void);

#endif /* ATTITUDE_H */
