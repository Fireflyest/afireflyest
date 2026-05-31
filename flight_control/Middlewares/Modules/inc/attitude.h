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

#endif /* ATTITUDE_H */
