#ifndef __EKF_STATE7_H
#define __EKF_STATE7_H

#include "arm_math.h"

// 状态: [q0,q1,q2,q3, bias_x,bias_y,bias_z, altitude, vel_z]
#define EKF_STATE_DIM  9
#define EKF_MEAS_DIM   5

// 状态索引
#define EKF_IDX_Q0   0
#define EKF_IDX_Q1   1
#define EKF_IDX_Q2   2
#define EKF_IDX_Q3   3
#define EKF_IDX_BX   4
#define EKF_IDX_BY   5
#define EKF_IDX_BZ   6
#define EKF_IDX_ALT  7
#define EKF_IDX_VZ   8

// 观测索引
#define EKF_OBS_AX   0
#define EKF_OBS_AY   1
#define EKF_OBS_AZ   2
#define EKF_OBS_ALT  3

// EKF结构体
typedef struct {
    // 状态向量 [q0,q1,q2,q3, bias_x,bias_y,bias_z, altitude]
    float32_t x[EKF_STATE_DIM];
    // 协方差矩阵 (行优先存储)
    float32_t P_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 P;
    // 过程噪声
    float32_t Q_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 Q;
    // 观测噪声
    float32_t R_data[EKF_MEAS_DIM * EKF_MEAS_DIM];
    arm_matrix_instance_f32 R;
    // 雅可比矩阵F
    float32_t F_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 F;
    // 观测雅可比H
    float32_t H_data[EKF_MEAS_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 H;
    // 卡尔曼增益K
    float32_t K_data[EKF_STATE_DIM * EKF_MEAS_DIM];
    arm_matrix_instance_f32 K;
    // 临时矩阵
    float32_t FP_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 FP;
    float32_t Ft_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 Ft;
    float32_t S_data[EKF_MEAS_DIM * EKF_MEAS_DIM];
    arm_matrix_instance_f32 S;
    float32_t S_inv_data[EKF_MEAS_DIM * EKF_MEAS_DIM];
    arm_matrix_instance_f32 S_inv;
    float32_t HP_data[EKF_MEAS_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 HP;
    float32_t HPHt_data[EKF_MEAS_DIM * EKF_MEAS_DIM];
    arm_matrix_instance_f32 HPHt;
    float32_t PHt_data[EKF_STATE_DIM * EKF_MEAS_DIM];
    arm_matrix_instance_f32 PHt;
    float32_t KH_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 KH;
    float32_t IKH_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 IKH;
    float32_t Ht_data[EKF_STATE_DIM * EKF_MEAS_DIM];
    arm_matrix_instance_f32 Ht;
    float32_t IKHt_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 IKHt;
    float32_t KR_data[EKF_STATE_DIM * EKF_MEAS_DIM];
    arm_matrix_instance_f32 KR;
    float32_t Kt_data[EKF_MEAS_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 Kt;
    float32_t KRKt_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 KRKt;
    float32_t P_mid_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 P_mid;
    // 观测向量
    float32_t z[EKF_MEAS_DIM];
    float32_t h[EKF_MEAS_DIM];
    float32_t y[EKF_MEAS_DIM];

    // 初始化标志
    uint8_t alt_initialized;
    uint8_t quat_initialized;
} EKF_Handle_t;

// 函数声明
void EKF_Init(EKF_Handle_t *ekf);
void EKF_Update(EKF_Handle_t *ekf, const float32_t accel[3], const float32_t gyro[3], const float32_t baro_altitude, float32_t dt);
void EKF_GetEuler(const EKF_Handle_t *ekf, float32_t *roll, float32_t *pitch, float32_t *yaw);
float32_t EKF_GetAltitude(const EKF_Handle_t *ekf);
float32_t EKF_GetVelocityZ(const EKF_Handle_t *ekf);

#endif /* __EKF_STATE7_H */