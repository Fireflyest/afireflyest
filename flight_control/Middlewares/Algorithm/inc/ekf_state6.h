#ifndef __EKF_STATE6_H
#define __EKF_STATE6_H

#include "arm_math.h"

// 状态向量维度: [q0, q1, q2, q3, bias_x, bias_y, bias_z]
#define EKF_STATE_DIM 7
#define EKF_MEAS_DIM 3

// EKF结构体 
typedef struct {
    // 状态向量
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
    // 临时矩阵 (避免重复分配)
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
    // 观测向量 
    float32_t z[EKF_MEAS_DIM];
    float32_t h[EKF_MEAS_DIM];
    float32_t y[EKF_MEAS_DIM];
} EKF_Handle_t;

// 函数声明
void EKF_Init(EKF_Handle_t *ekf);
void EKF_Update(EKF_Handle_t *ekf, const float32_t accel[3], const float32_t gyro[3], float32_t dt);
void EKF_GetEuler(const EKF_Handle_t *ekf, float32_t *roll, float32_t *pitch, float32_t *yaw); 

#endif /* __EKF_STATE_H */