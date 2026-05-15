#include "ekf_state6.h"

#include <string.h>
#include <math.h>
#include "arm_math.h"

// ==================================
// 内部辅助函数
// ==================================
static void Quaternion_Multiply(const float32_t q1[4], const float32_t q2[4], float32_t q_out[4])
{
    q_out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void Quaternion_Normalize(float32_t q[4])
{
    float32_t norm;
    arm_status status = arm_sqrt_f32(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3], &norm);
    if (status == ARM_MATH_SUCCESS && norm > 1e-6f) {
        float32_t inv_norm = 1.0f / norm;
        q[0] *= inv_norm;
        q[1] *= inv_norm;
        q[2] *= inv_norm;
        q[3] *= inv_norm;
    }
}

static void State_Transition(EKF_Handle_t *ekf, const float32_t gyro[3], float32_t dt)
{
    float32_t *x = ekf->x;

    float32_t omega[3] = {
        gyro[0] - x[4],
        gyro[1] - x[5],
        gyro[2] - x[6]
    };

    float32_t omega_q[4] = {0.0f, omega[0], omega[1], omega[2]};
    float32_t q_dot[4];

    Quaternion_Multiply(x, omega_q, q_dot);      // q_dot = q * omega_q
    arm_scale_f32(q_dot, 0.5f * dt, q_dot, 4);   // q_dot *= 0.5*dt
    arm_add_f32(x, q_dot, x, 4);                  // x += q_dot
    Quaternion_Normalize(x);
}

static void Compute_JacobianF(EKF_Handle_t *ekf, const float32_t gyro[3], float32_t dt)
{
    float32_t *x = ekf->x;
    float32_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float32_t wx = gyro[0] - x[4];
    float32_t wy = gyro[1] - x[5];
    float32_t wz = gyro[2] - x[6];
    float32_t *F = ekf->F_data;

    memset(F, 0, EKF_STATE_DIM * EKF_STATE_DIM * sizeof(float32_t));
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        F[i * EKF_STATE_DIM + i] = 1.0f;
    }

    float32_t half_dt = 0.5f * dt;
    F[0*7+1] = -half_dt * wx;
    F[0*7+2] = -half_dt * wy;
    F[0*7+3] = -half_dt * wz;
    F[1*7+0] =  half_dt * wx;
    F[1*7+2] =  half_dt * wz;
    F[1*7+3] = -half_dt * wy;
    F[2*7+0] =  half_dt * wy;
    F[2*7+1] = -half_dt * wz;
    F[2*7+3] =  half_dt * wx;
    F[3*7+0] =  half_dt * wz;
    F[3*7+1] =  half_dt * wy;
    F[3*7+2] = -half_dt * wx;

    F[0*7+4] =  half_dt * q1;
    F[0*7+5] =  half_dt * q2;
    F[0*7+6] =  half_dt * q3;
    F[1*7+4] = -half_dt * q0;
    F[1*7+5] =  half_dt * q3;
    F[1*7+6] = -half_dt * q2;
    F[2*7+4] = -half_dt * q3;
    F[2*7+5] = -half_dt * q0;
    F[2*7+6] =  half_dt * q1;
    F[3*7+4] =  half_dt * q2;
    F[3*7+5] = -half_dt * q1;
    F[3*7+6] = -half_dt * q0;
}

static void Compute_JacobianH(EKF_Handle_t *ekf)
{
    float32_t q0 = ekf->x[0], q1 = ekf->x[1], q2 = ekf->x[2], q3 = ekf->x[3];
    float32_t *H = ekf->H_data;
    memset(H, 0, EKF_MEAS_DIM * EKF_STATE_DIM * sizeof(float32_t));

    H[0*7+0] = -2.0f*q2;
    H[0*7+1] =  2.0f*q3;
    H[0*7+2] = -2.0f*q0;
    H[0*7+3] =  2.0f*q1;

    H[1*7+0] =  2.0f*q1;
    H[1*7+1] =  2.0f*q0;
    H[1*7+2] =  2.0f*q3;
    H[1*7+3] =  2.0f*q2;

    H[2*7+0] =  2.0f*q0;
    H[2*7+1] = -2.0f*q1;
    H[2*7+2] = -2.0f*q2;
    H[2*7+3] =  2.0f*q3;
}

static void Observation_Model(const float32_t x[7], float32_t h[3])
{
    float32_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    h[0] = 2.0f * (q1*q3 - q0*q2);
    h[1] = 2.0f * (q0*q1 + q2*q3);
    h[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

static arm_status Matrix_Inverse3x3(arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst)
{
    float32_t *m = src->pData;
    float32_t a = m[0], b = m[1], c = m[2];
    float32_t d = m[3], e = m[4], f = m[5];
    float32_t g = m[6], h = m[7], i = m[8];

    float32_t det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    if (fabsf(det) < 1e-6f) {
        return ARM_MATH_SINGULAR;
    }

    float32_t inv_det = 1.0f / det;
    float32_t *out = dst->pData;
    out[0] =  (e*i - f*h) * inv_det;
    out[1] =  (c*h - b*i) * inv_det;
    out[2] =  (b*f - c*e) * inv_det;
    out[3] =  (f*g - d*i) * inv_det;
    out[4] =  (a*i - c*g) * inv_det;
    out[5] =  (c*d - a*f) * inv_det;
    out[6] =  (d*h - e*g) * inv_det;
    out[7] =  (b*g - a*h) * inv_det;
    out[8] =  (a*e - b*d) * inv_det;

    return ARM_MATH_SUCCESS;
}

// ==================================
// 公开接口
// ==================================
void EKF_Init(EKF_Handle_t *ekf)
{
    memset(ekf, 0, sizeof(EKF_Handle_t));
    ekf->x[0] = 1.0f; // 单位四元数

    arm_mat_init_f32(&ekf->P, EKF_STATE_DIM, EKF_STATE_DIM, ekf->P_data);
    arm_mat_init_f32(&ekf->Q, EKF_STATE_DIM, EKF_STATE_DIM, ekf->Q_data);
    arm_mat_init_f32(&ekf->R, EKF_MEAS_DIM, EKF_MEAS_DIM, ekf->R_data);
    arm_mat_init_f32(&ekf->F, EKF_STATE_DIM, EKF_STATE_DIM, ekf->F_data);
    arm_mat_init_f32(&ekf->H, EKF_MEAS_DIM, EKF_STATE_DIM, ekf->H_data);
    arm_mat_init_f32(&ekf->K, EKF_STATE_DIM, EKF_MEAS_DIM, ekf->K_data);
    arm_mat_init_f32(&ekf->FP, EKF_STATE_DIM, EKF_STATE_DIM, ekf->FP_data);
    arm_mat_init_f32(&ekf->Ft, EKF_STATE_DIM, EKF_STATE_DIM, ekf->Ft_data);
    arm_mat_init_f32(&ekf->S, EKF_MEAS_DIM, EKF_MEAS_DIM, ekf->S_data);
    arm_mat_init_f32(&ekf->S_inv, EKF_MEAS_DIM, EKF_MEAS_DIM, ekf->S_inv_data);
    arm_mat_init_f32(&ekf->HP, EKF_MEAS_DIM, EKF_STATE_DIM, ekf->HP_data);
    arm_mat_init_f32(&ekf->PHt, EKF_STATE_DIM, EKF_MEAS_DIM, ekf->PHt_data);
    arm_mat_init_f32(&ekf->HPHt, EKF_MEAS_DIM, EKF_MEAS_DIM, ekf->HPHt_data);
    arm_mat_init_f32(&ekf->KH, EKF_STATE_DIM, EKF_STATE_DIM, ekf->KH_data);
    arm_mat_init_f32(&ekf->IKH, EKF_STATE_DIM, EKF_STATE_DIM, ekf->IKH_data);

    for (int i = 0; i < 4; i++) {
        ekf->P_data[i * EKF_STATE_DIM + i] = 0.01f;
        ekf->Q_data[i * EKF_STATE_DIM + i] = 1e-4f;
    }
    for (int i = 4; i < 7; i++) {
        ekf->P_data[i * EKF_STATE_DIM + i] = 0.001f;
        ekf->Q_data[i * EKF_STATE_DIM + i] = 1e-6f;
    }
    for (int i = 0; i < 3; i++) {
        ekf->R_data[i * 3 + i] = 0.5f;
    }
}

void EKF_Update(EKF_Handle_t *ekf, const float32_t accel[3], const float32_t gyro[3], float32_t dt)
{
    arm_status status;

    // 预测
    State_Transition(ekf, gyro, dt);
    Compute_JacobianF(ekf, gyro, dt);

    status = arm_mat_mult_f32(&ekf->F, &ekf->P, &ekf->FP);
    status = arm_mat_trans_f32(&ekf->F, &ekf->Ft);
    status = arm_mat_mult_f32(&ekf->FP, &ekf->Ft, &ekf->P);
    status = arm_mat_add_f32(&ekf->P, &ekf->Q, &ekf->P);

    // 更新
    float32_t accel_norm;
    status = arm_sqrt_f32(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2], &accel_norm);
    if (accel_norm < 1e-6f) return;

    float32_t inv_norm = 1.0f / accel_norm;

    // NED 环境下：加速度计测量的力向上。如果用测量数据对比重力(Down)，则需反转
    ekf->z[0] = -accel[0] * inv_norm;
    ekf->z[1] = -accel[1] * inv_norm;
    ekf->z[2] = -accel[2] * inv_norm;

    Observation_Model(ekf->x, ekf->h);
    arm_sub_f32(ekf->z, ekf->h, ekf->y, 3);

    Compute_JacobianH(ekf);

    status = arm_mat_mult_f32(&ekf->H, &ekf->P, &ekf->HP);
    static arm_matrix_instance_f32 Ht;
    static float32_t Ht_data[EKF_STATE_DIM * EKF_MEAS_DIM];
    arm_mat_init_f32(&Ht, EKF_STATE_DIM, EKF_MEAS_DIM, Ht_data);
    status = arm_mat_trans_f32(&ekf->H, &Ht);
    status = arm_mat_mult_f32(&ekf->HP, &Ht, &ekf->S);
    status = arm_mat_add_f32(&ekf->S, &ekf->R, &ekf->S);

    status = Matrix_Inverse3x3(&ekf->S, &ekf->S_inv);
    if (status != ARM_MATH_SUCCESS) return;

    status = arm_mat_mult_f32(&ekf->P, &Ht, &ekf->PHt);
    status = arm_mat_mult_f32(&ekf->PHt, &ekf->S_inv, &ekf->K);

    float32_t Ky[EKF_STATE_DIM];
    arm_matrix_instance_f32 Ky_mat, y_mat;
    arm_mat_init_f32(&Ky_mat, EKF_STATE_DIM, 1, Ky);
    arm_mat_init_f32(&y_mat, EKF_MEAS_DIM, 1, ekf->y);

    status = arm_mat_mult_f32(&ekf->K, &y_mat, &Ky_mat);
    arm_add_f32(ekf->x, Ky, ekf->x, EKF_STATE_DIM);
    Quaternion_Normalize(ekf->x);

    status = arm_mat_mult_f32(&ekf->K, &ekf->H, &ekf->KH);

    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            if (i == j) {
                ekf->IKH_data[i * EKF_STATE_DIM + j] = 1.0f - ekf->KH_data[i * EKF_STATE_DIM + j];
            } else {
                ekf->IKH_data[i * EKF_STATE_DIM + j] = -ekf->KH_data[i * EKF_STATE_DIM + j];
            }
        }
    }

    float32_t P_temp_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 P_temp;
    arm_mat_init_f32(&P_temp, EKF_STATE_DIM, EKF_STATE_DIM, P_temp_data);
    memcpy(P_temp_data, ekf->P_data, sizeof(P_temp_data));
    status = arm_mat_mult_f32(&ekf->IKH, &P_temp, &ekf->P);
}

void EKF_GetEuler(const EKF_Handle_t *ekf, float32_t *roll, float32_t *pitch, float32_t *yaw)
{
    float32_t q0 = ekf->x[0], q1 = ekf->x[1], q2 = ekf->x[2], q3 = ekf->x[3];

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    *pitch = asinf(2.0f*(q0*q2 - q3*q1));
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}
