#include "ekf_state7.h"

#include <math.h>
#include <string.h>
#include "arm_math.h"

#define GRAVITY 9.81f

/**
 *  坐标系定义 (FRD / NED)
 *  ─────────────────────────────────────────────────────────────────────────
 *
 *    机体坐标系 (Body Frame) — FRD (Front-Right-Down)
 *    X 轴：机头方向 (Forward)
 *    Y 轴：右侧方向 (Right)
 *    Z 轴：下方方向 (Down)
 *    陀螺仪/加速度计输出直接对应此坐标系
 *
 *    世界坐标系 (World Frame) — NED (North-East-Down)
 *    X 轴：北 (North)
 *    Y 轴：东 (East)
 *    Z 轴：地 (Down) ← 向下为正
 *    重力方向：+Z（向下），大小 = 9.81 m/s²
 *    因此：起飞 = alt 减小, vz 为负
 *
 *    旋转关系：v_body = R(q) · v_world
 *    其中 R(q) 为四元数对应的旋转矩阵（世界→机体）
 *
 *    ★ 重要：body→world 变换使用 R^T，即 R 的第三列 R[:,2]
 *      R[:,2] = [2(q1q3+q0q2), 2(q2q3-q0q1), q0²-q1²-q2²+q3²]
 *      R[2,:] = [2(q1q3-q0q2), 2(q0q1+q2q3), q0²-q1²-q2²+q3²]  ← 不同！
 */

// ==================================
// 安全辅助宏
// ==================================
#define CHECK_MAT_STATUS(expr)          \
    do {                                \
        status = (expr);                \
        if (status != ARM_MATH_SUCCESS) \
            goto _ekf_health_check;     \
    } while (0)

// ==================================
// 四元数运算
// ==================================
static void Quaternion_Multiply(const float32_t q1[4], const float32_t q2[4], float32_t q_out[4]) {
    q_out[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    q_out[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    q_out[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    q_out[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

static void Quaternion_Normalize(float32_t q[4]) {
    /* ★ FIX: 增加 NaN/退化保护 */
    float32_t norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!isfinite(norm_sq) || norm_sq < 1e-12f) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }
    float32_t norm;
    arm_sqrt_f32(norm_sq, &norm);
    float32_t inv_norm = 1.0f / norm;
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
}

// ==================================
// 状态转移 (只更新状态, 不用作 Jacobian 输入)
// ==================================
static void State_Transition(EKF_Handle_t* ekf, const float32_t gyro[3], const float32_t accel[3], float32_t dt) {
    float32_t* x = ekf->x;

    // 角速度 = 陀螺仪 - 偏置
    float32_t omega[3] = {
        gyro[0] - x[EKF_IDX_BX],
        gyro[1] - x[EKF_IDX_BY],
        gyro[2] - x[EKF_IDX_BZ]};

    // 四元数更新
    float32_t omega_q[4] = {0.0f, omega[0], omega[1], omega[2]};
    float32_t q_dot[4];
    Quaternion_Multiply(x, omega_q, q_dot);
    arm_scale_f32(q_dot, 0.5f * dt, q_dot, 4);
    arm_add_f32(x, q_dot, x, 4);
    Quaternion_Normalize(x);

    float32_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

    /* ★ FIX: 使用 R[:,2]（旋转矩阵第三列）将机体加速度转换到世界坐标系
     *   R[:,2] = [2(q1q3+q0q2), 2(q2q3-q0q1), q0²-q1²-q2²+q3²]
     *   原代码错误使用了 R[2,:]（第三行），符号不同
     */
    float32_t R02 = 2.0f * (q1 * q3 + q0 * q2);
    float32_t R12 = 2.0f * (q2 * q3 - q0 * q1);
    float32_t R22 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float32_t az_world = R02 * accel[0] + R12 * accel[1] + R22 * accel[2] + GRAVITY;

    x[EKF_IDX_VZ] += az_world * dt;
    x[EKF_IDX_ALT] += x[EKF_IDX_VZ] * dt;
}

// ==================================
// Jacobian F (用旧四元数计算)
// ==================================
static void Compute_JacobianF(EKF_Handle_t* ekf, const float32_t gyro[3], const float32_t accel[3], float32_t dt, const float32_t q_old[4]) {
    float32_t q0 = q_old[0], q1 = q_old[1], q2 = q_old[2], q3 = q_old[3];
    float32_t wx = gyro[0] - ekf->x[EKF_IDX_BX];
    float32_t wy = gyro[1] - ekf->x[EKF_IDX_BY];
    float32_t wz = gyro[2] - ekf->x[EKF_IDX_BZ];
    float32_t ax = accel[0], ay = accel[1], az = accel[2];
    float32_t* F = ekf->F_data;

    // 初始化为单位矩阵
    memset(F, 0, EKF_STATE_DIM * EKF_STATE_DIM * sizeof(float32_t));
    for (int i = 0; i < EKF_STATE_DIM; i++)
        F[i * EKF_STATE_DIM + i] = 1.0f;

    float32_t half_dt = 0.5f * dt;

    // ∂q/∂q
    F[0 * EKF_STATE_DIM + 1] = -half_dt * wx;
    F[0 * EKF_STATE_DIM + 2] = -half_dt * wy;
    F[0 * EKF_STATE_DIM + 3] = -half_dt * wz;
    F[1 * EKF_STATE_DIM + 0] = half_dt * wx;
    F[1 * EKF_STATE_DIM + 2] = half_dt * wz;
    F[1 * EKF_STATE_DIM + 3] = -half_dt * wy;
    F[2 * EKF_STATE_DIM + 0] = half_dt * wy;
    F[2 * EKF_STATE_DIM + 1] = -half_dt * wz;
    F[2 * EKF_STATE_DIM + 3] = half_dt * wx;
    F[3 * EKF_STATE_DIM + 0] = half_dt * wz;
    F[3 * EKF_STATE_DIM + 1] = half_dt * wy;
    F[3 * EKF_STATE_DIM + 2] = -half_dt * wx;

    /* ★ FIX: ∂q/∂bias — 修正 3 处符号错误
     *   推导: q_new = q + 0.5·dt·(q⊗[0,ω]), ω = gyro - bias
     *   ∂q_new/∂bias = -0.5·dt · ∂(q⊗[0,ω])/∂ω
     */
    F[0 * EKF_STATE_DIM + 4] = half_dt * q1;
    F[0 * EKF_STATE_DIM + 5] = half_dt * q2;
    F[0 * EKF_STATE_DIM + 6] = half_dt * q3;
    F[1 * EKF_STATE_DIM + 4] = -half_dt * q0;
    F[1 * EKF_STATE_DIM + 5] = half_dt * q3; /* ★ 原为 -half_dt * q3 */
    F[1 * EKF_STATE_DIM + 6] = -half_dt * q2;
    F[2 * EKF_STATE_DIM + 4] = -half_dt * q3; /* ★ 原为 +half_dt * q3 */
    F[2 * EKF_STATE_DIM + 5] = -half_dt * q0;
    F[2 * EKF_STATE_DIM + 6] = half_dt * q1;
    F[3 * EKF_STATE_DIM + 4] = half_dt * q2;
    F[3 * EKF_STATE_DIM + 5] = -half_dt * q1; /* ★ 原为 +half_dt * q1 */
    F[3 * EKF_STATE_DIM + 6] = -half_dt * q0;

    // ∂alt/∂vz = dt
    F[7 * EKF_STATE_DIM + 8] = dt;

    /* ★ FIX: ∂vz/∂q — 更新为匹配修正后的 R[:,2] 状态转移
     *   az_world = R02*ax + R12*ay + R22*az + g
     *   R02 = 2(q1q3+q0q2), R12 = 2(q2q3-q0q1), R22 = q0²-q1²-q2²+q3²
     */
    F[8 * EKF_STATE_DIM + 0] = dt * 2.0f * (q2 * ax - q1 * ay + q0 * az);
    F[8 * EKF_STATE_DIM + 1] = dt * 2.0f * (q3 * ax - q0 * ay - q1 * az);
    F[8 * EKF_STATE_DIM + 2] = dt * 2.0f * (q0 * ax + q3 * ay - q2 * az);
    F[8 * EKF_STATE_DIM + 3] = dt * 2.0f * (q1 * ax + q2 * ay + q3 * az);
}

// ==================================
// 观测雅可比 H
// ==================================
static void Compute_JacobianH(EKF_Handle_t* ekf) {
    float32_t q0 = ekf->x[0], q1 = ekf->x[1], q2 = ekf->x[2], q3 = ekf->x[3];
    float32_t* H = ekf->H_data;
    memset(H, 0, EKF_MEAS_DIM * EKF_STATE_DIM * sizeof(float32_t));

    /* ★ FIX: 使用 R[:,2] 的偏导数（原代码使用了 R[2,:] 的偏导数）
     *   h[0] = 2(q1q3 + q0q2)   ∂h[0]/∂q = [ 2q2,  2q3,  2q0,  2q1]
     *   h[1] = 2(q2q3 - q0q1)   ∂h[1]/∂q = [-2q1, -2q0,  2q3,  2q2]
     *   h[2] = q0²-q1²-q2²+q3²  ∂h[2]/∂q = [ 2q0, -2q1, -2q2,  2q3]
     */
    H[0 * EKF_STATE_DIM + 0] = 2.0f * q2; /* ★ 原为 -2*q2 */
    H[0 * EKF_STATE_DIM + 1] = 2.0f * q3;
    H[0 * EKF_STATE_DIM + 2] = 2.0f * q0; /* ★ 原为 -2*q0 */
    H[0 * EKF_STATE_DIM + 3] = 2.0f * q1;

    H[1 * EKF_STATE_DIM + 0] = -2.0f * q1; /* ★ 原为 +2*q1 */
    H[1 * EKF_STATE_DIM + 1] = -2.0f * q0; /* ★ 原为 +2*q0 */
    H[1 * EKF_STATE_DIM + 2] = 2.0f * q3;
    H[1 * EKF_STATE_DIM + 3] = 2.0f * q2;

    H[2 * EKF_STATE_DIM + 0] = 2.0f * q0;
    H[2 * EKF_STATE_DIM + 1] = -2.0f * q1;
    H[2 * EKF_STATE_DIM + 2] = -2.0f * q2;
    H[2 * EKF_STATE_DIM + 3] = 2.0f * q3;

    H[3 * EKF_STATE_DIM + 7] = 1.0f;
    H[4 * EKF_STATE_DIM + 8] = 1.0f;
}

// ==================================
// 观测模型
// ==================================
static void Observation_Model(const float32_t x[EKF_STATE_DIM], float32_t h[EKF_MEAS_DIM]) {
    float32_t q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

    /* ★ FIX: 使用 R[:,2]（旋转矩阵第三列）
     *   加速度计测量值 z = -accel/||accel|| 在静止时等于 R[:,2]
     *   h 必须预测相同的量
     *   R[:,2] = [2(q1q3+q0q2), 2(q2q3-q0q1), q0²-q1²-q2²+q3²]
     */
    h[0] = 2.0f * (q1 * q3 + q0 * q2); /* ★ 原为 2*(q1q3 - q0q2) */
    h[1] = 2.0f * (q2 * q3 - q0 * q1); /* ★ 原为 2*(q0q1 + q2q3) */
    h[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    h[3] = x[EKF_IDX_ALT];
    h[4] = x[EKF_IDX_VZ];
}

// ==================================
// 4×4 矩阵求逆
// ==================================
static arm_status Matrix_Inverse4x4(arm_matrix_instance_f32* src, arm_matrix_instance_f32* dst) {
    float32_t* m = src->pData;
    int n = 4;
    float32_t aug[4][8];

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i][j] = m[i * n + j];
            aug[i][j + n] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int col = 0; col < n; col++) {
        int best = col;
        for (int row = col + 1; row < n; row++)
            if (fabsf(aug[row][col]) > fabsf(aug[best][col]))
                best = row;
        if (best != col)
            for (int j = 0; j < 2 * n; j++) {
                float32_t t = aug[col][j];
                aug[col][j] = aug[best][j];
                aug[best][j] = t;
            }

        float32_t pivot = aug[col][col];
        if (fabsf(pivot) < 1e-12f)
            return ARM_MATH_SINGULAR;

        float32_t inv_pivot = 1.0f / pivot;
        for (int j = 0; j < 2 * n; j++)
            aug[col][j] *= inv_pivot;

        for (int row = 0; row < n; row++) {
            if (row == col)
                continue;
            float32_t f = aug[row][col];
            for (int j = 0; j < 2 * n; j++)
                aug[row][j] -= f * aug[col][j];
        }
    }

    float32_t* out = dst->pData;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            out[i * n + j] = aug[i][j + n];

    return ARM_MATH_SUCCESS;
}

// ==================================
// 5×5 矩阵求逆
// ==================================
static arm_status Matrix_Inverse5x5(arm_matrix_instance_f32* src, arm_matrix_instance_f32* dst) {
    float32_t* m = src->pData;
    int n = 5;
    float32_t aug[5][10];

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i][j] = m[i * n + j];
            aug[i][j + n] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int col = 0; col < n; col++) {
        int best = col;
        for (int row = col + 1; row < n; row++)
            if (fabsf(aug[row][col]) > fabsf(aug[best][col]))
                best = row;
        if (best != col)
            for (int j = 0; j < 2 * n; j++) {
                float32_t t = aug[col][j];
                aug[col][j] = aug[best][j];
                aug[best][j] = t;
            }

        float32_t pivot = aug[col][col];
        if (fabsf(pivot) < 1e-12f)
            return ARM_MATH_SINGULAR;

        float32_t inv_pivot = 1.0f / pivot;
        for (int j = 0; j < 2 * n; j++)
            aug[col][j] *= inv_pivot;

        for (int row = 0; row < n; row++) {
            if (row == col)
                continue;
            float32_t f = aug[row][col];
            for (int j = 0; j < 2 * n; j++)
                aug[row][j] -= f * aug[col][j];
        }
    }

    float32_t* out = dst->pData;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            out[i * n + j] = aug[i][j + n];

    return ARM_MATH_SUCCESS;
}

// ==================================
// P 矩阵健康检查与恢复
// ==================================
static void EKF_ResetCovariance(EKF_Handle_t* ekf) {
    memset(ekf->P_data, 0, EKF_STATE_DIM * EKF_STATE_DIM * sizeof(float32_t));
    ekf->P_data[0 * EKF_STATE_DIM + 0] = 0.01f;
    ekf->P_data[1 * EKF_STATE_DIM + 1] = 0.01f;
    ekf->P_data[2 * EKF_STATE_DIM + 2] = 0.01f;
    ekf->P_data[3 * EKF_STATE_DIM + 3] = 0.01f;
    ekf->P_data[4 * EKF_STATE_DIM + 4] = 0.001f;
    ekf->P_data[5 * EKF_STATE_DIM + 5] = 0.001f;
    ekf->P_data[6 * EKF_STATE_DIM + 6] = 0.001f;
    ekf->P_data[7 * EKF_STATE_DIM + 7] = 100.0f;
    ekf->P_data[8 * EKF_STATE_DIM + 8] = 10.0f;
}

// ==================================
// 初始化
// ==================================
void EKF_Init(EKF_Handle_t* ekf) {
    memset(ekf, 0, sizeof(EKF_Handle_t));

    // 矩阵实例初始化
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
    arm_mat_init_f32(&ekf->Ht, EKF_STATE_DIM, EKF_MEAS_DIM, ekf->Ht_data);
    arm_mat_init_f32(&ekf->IKHt, EKF_STATE_DIM, EKF_STATE_DIM, ekf->IKHt_data);
    arm_mat_init_f32(&ekf->KR, EKF_STATE_DIM, EKF_MEAS_DIM, ekf->KR_data);
    arm_mat_init_f32(&ekf->Kt, EKF_MEAS_DIM, EKF_STATE_DIM, ekf->Kt_data);
    arm_mat_init_f32(&ekf->KRKt, EKF_STATE_DIM, EKF_STATE_DIM, ekf->KRKt_data);
    arm_mat_init_f32(&ekf->P_mid, EKF_STATE_DIM, EKF_STATE_DIM, ekf->P_mid_data);

    // 初始状态: 单位四元数
    ekf->x[0] = 1.0f;

    // 协方差 P
    ekf->P_data[0 * EKF_STATE_DIM + 0] = 0.01f;
    ekf->P_data[1 * EKF_STATE_DIM + 1] = 0.01f;
    ekf->P_data[2 * EKF_STATE_DIM + 2] = 0.01f;
    ekf->P_data[3 * EKF_STATE_DIM + 3] = 0.01f;
    ekf->P_data[4 * EKF_STATE_DIM + 4] = 0.001f;
    ekf->P_data[5 * EKF_STATE_DIM + 5] = 0.001f;
    ekf->P_data[6 * EKF_STATE_DIM + 6] = 0.001f;
    ekf->P_data[7 * EKF_STATE_DIM + 7] = 100.0f;
    ekf->P_data[8 * EKF_STATE_DIM + 8] = 10.0f;

    // 过程噪声 Q
    ekf->Q_data[0 * EKF_STATE_DIM + 0] = 1e-4f;
    ekf->Q_data[1 * EKF_STATE_DIM + 1] = 1e-4f;
    ekf->Q_data[2 * EKF_STATE_DIM + 2] = 1e-4f;
    ekf->Q_data[3 * EKF_STATE_DIM + 3] = 1e-4f;
    ekf->Q_data[4 * EKF_STATE_DIM + 4] = 1e-6f;
    ekf->Q_data[5 * EKF_STATE_DIM + 5] = 1e-6f;
    ekf->Q_data[6 * EKF_STATE_DIM + 6] = 1e-6f;
    ekf->Q_data[7 * EKF_STATE_DIM + 7] = 0.0001f;
    ekf->Q_data[8 * EKF_STATE_DIM + 8] = 0.01f;

    // 观测噪声 R
    ekf->R_data[0 * EKF_MEAS_DIM + 0] = 0.5f;
    ekf->R_data[1 * EKF_MEAS_DIM + 1] = 0.5f;
    ekf->R_data[2 * EKF_MEAS_DIM + 2] = 0.5f;
    ekf->R_data[3 * EKF_MEAS_DIM + 3] = 30.0f;
    /* 注意：z[4] 对应垂直速度测量，当前无外部 vz 输入时 z[4]=0
     * R[4][4]=0.01 意味着 EKF 会强力约束 vz→0，可能在垂直运动时产生问题
     * 若无 vz 测量源，建议增大此值（如 100.0f）以降低约束强度 */
    ekf->R_data[4 * EKF_MEAS_DIM + 4] = 0.01f;

    ekf->alt_initialized = 0;
    ekf->quat_initialized = 0;
}

// ==================================
// EKF 更新
// ==================================
void EKF_Update(EKF_Handle_t* ekf, const float32_t accel[3], const float32_t gyro[3], float32_t baro_altitude, float32_t dt) {
    arm_status status;

    /* ★ FIX: 输入数据合法性检查 */
    for (int i = 0; i < 3; i++) {
        if (!isfinite(accel[i]) || !isfinite(gyro[i]))
            return;
    }
    if (!isfinite(baro_altitude) || dt <= 0.0f || dt > 0.1f)
        return;

    if (!ekf->alt_initialized) {
        ekf->x[EKF_IDX_ALT] = -baro_altitude;
        ekf->alt_initialized = 1;
    }

    if (!ekf->quat_initialized) {
        float32_t ax = accel[0], ay = accel[1], az = accel[2];
        float32_t acc_norm;
        arm_sqrt_f32(ax * ax + ay * ay + az * az, &acc_norm);
        if (acc_norm > 1e-6f) {
            float32_t inv = 1.0f / acc_norm;
            ax *= inv;
            ay *= inv;
            az *= inv;

            /* ★ FIX: 修正初始欧拉角符号
             *   加速度计归一化后 = -R[:,2]
             *   ax ≈ -sin(pitch), ay ≈ sin(roll), az ≈ -cos(roll)*cos(pitch)
             *   roll  = atan2(ay, -az)   ← 原为 atan2(-ay, -az)
             *   pitch = asinf(-ax)        ← 原为 asinf(ax)
             */
            float32_t roll = atan2f(ay, -az);
            float32_t pitch = asinf(-ax);

            float32_t cr = cosf(roll * 0.5f);
            float32_t sr = sinf(roll * 0.5f);
            float32_t cp = cosf(pitch * 0.5f);
            float32_t sp = sinf(pitch * 0.5f);

            ekf->x[0] = cr * cp;
            ekf->x[1] = sr * cp;
            ekf->x[2] = cr * sp;
            ekf->x[3] = sr * sp;
        }
        ekf->quat_initialized = 1;
    }

    // ===== 保存旧四元数 (用于 Jacobian 和回滚) =====
    float32_t q_old[4] = {
        ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3]};
    float32_t x_old[EKF_STATE_DIM];
    memcpy(x_old, ekf->x, sizeof(x_old));

    // ===== 预测 =====
    State_Transition(ekf, gyro, accel, dt);
    Compute_JacobianF(ekf, gyro, accel, dt, q_old);

    /* ★ FIX: 检查矩阵运算返回值 */
    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->F, &ekf->P, &ekf->FP));
    CHECK_MAT_STATUS(arm_mat_trans_f32(&ekf->F, &ekf->Ft));
    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->FP, &ekf->Ft, &ekf->P));
    CHECK_MAT_STATUS(arm_mat_add_f32(&ekf->P, &ekf->Q, &ekf->P));

    /* ★ FIX: 检查预测后状态是否合法 */
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        if (!isfinite(ekf->x[i])) {
            memcpy(ekf->x, x_old, sizeof(x_old));
            EKF_ResetCovariance(ekf);
            return;
        }
    }

    // ===== 更新 =====
    float32_t accel_norm;
    arm_sqrt_f32(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2], &accel_norm);
    if (accel_norm < 1e-6f)
        goto _ekf_health_check;

    float32_t inv_norm = 1.0f / accel_norm;

    ekf->z[0] = -accel[0] * inv_norm;
    ekf->z[1] = -accel[1] * inv_norm;
    ekf->z[2] = -accel[2] * inv_norm;
    ekf->z[3] = -baro_altitude;
    ekf->z[4] = 0.0f; /* ★ FIX: 显式初始化，原代码遗漏 */

    Observation_Model(ekf->x, ekf->h);
    arm_sub_f32(ekf->z, ekf->h, ekf->y, EKF_MEAS_DIM);
    Compute_JacobianH(ekf);

    /* ═══════════════════════════════════════════════════════
     *  阻断气压计对姿态的影响
     * ═══════════════════════════════════════════════════════ */
    for (int i = 0; i < 7; i++) {
        ekf->P_data[i * EKF_STATE_DIM + 7] = 0.0f;
        ekf->P_data[i * EKF_STATE_DIM + 8] = 0.0f;
        ekf->P_data[7 * EKF_STATE_DIM + i] = 0.0f;
        ekf->P_data[8 * EKF_STATE_DIM + i] = 0.0f;
    }

    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->H, &ekf->P, &ekf->HP));
    CHECK_MAT_STATUS(arm_mat_trans_f32(&ekf->H, &ekf->Ht));
    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->HP, &ekf->Ht, &ekf->S));
    CHECK_MAT_STATUS(arm_mat_add_f32(&ekf->S, &ekf->R, &ekf->S));

    status = Matrix_Inverse5x5(&ekf->S, &ekf->S_inv);
    if (status != ARM_MATH_SUCCESS) {
        /* ★ FIX: S 求逆失败时不再直接 return
         *   跳过测量更新，保留预测后的状态，继续做 P 健康检查 */
        goto _ekf_health_check;
    }

    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->P, &ekf->Ht, &ekf->PHt));
    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->PHt, &ekf->S_inv, &ekf->K));

    float32_t Ky[EKF_STATE_DIM];
    arm_matrix_instance_f32 Ky_mat, y_mat;
    arm_mat_init_f32(&Ky_mat, EKF_STATE_DIM, 1, Ky);
    arm_mat_init_f32(&y_mat, EKF_MEAS_DIM, 1, ekf->y);

    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->K, &y_mat, &Ky_mat));

    /* ★ FIX: 检查 Ky 是否合法后再更新状态 */
    {
        uint8_t ky_valid = 1;
        for (int i = 0; i < EKF_STATE_DIM; i++) {
            if (!isfinite(Ky[i])) {
                ky_valid = 0;
                break;
            }
        }
        if (ky_valid) {
            arm_add_f32(ekf->x, Ky, ekf->x, EKF_STATE_DIM);
            Quaternion_Normalize(ekf->x);
        }
    }

    // ===== Joseph 协方差更新 =====
    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->K, &ekf->H, &ekf->KH));

    for (int i = 0; i < EKF_STATE_DIM; i++)
        for (int j = 0; j < EKF_STATE_DIM; j++)
            ekf->IKH_data[i * EKF_STATE_DIM + j] =
                (i == j) ? 1.0f - ekf->KH_data[i * EKF_STATE_DIM + j]
                         : -ekf->KH_data[i * EKF_STATE_DIM + j];

    float32_t P_old_data[EKF_STATE_DIM * EKF_STATE_DIM];
    arm_matrix_instance_f32 P_old;
    arm_mat_init_f32(&P_old, EKF_STATE_DIM, EKF_STATE_DIM, P_old_data);
    memcpy(P_old_data, ekf->P_data, sizeof(P_old_data));

    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->IKH, &P_old, &ekf->P_mid));
    CHECK_MAT_STATUS(arm_mat_trans_f32(&ekf->IKH, &ekf->IKHt));
    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->P_mid, &ekf->IKHt, &ekf->P));

    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->K, &ekf->R, &ekf->KR));
    CHECK_MAT_STATUS(arm_mat_trans_f32(&ekf->K, &ekf->Kt));
    CHECK_MAT_STATUS(arm_mat_mult_f32(&ekf->KR, &ekf->Kt, &ekf->KRKt));
    CHECK_MAT_STATUS(arm_mat_add_f32(&ekf->P, &ekf->KRKt, &ekf->P));

_ekf_health_check:
    /* ★ FIX: P 矩阵对角线健康检查 — 始终执行 */
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        float32_t diag = ekf->P_data[i * EKF_STATE_DIM + i];
        if (!isfinite(diag) || diag < 0.0f || diag > 1e6f) {
            EKF_ResetCovariance(ekf);
            break;
        }
    }

    /* ★ FIX: 最终状态 NaN 检查 — 回滚到预测前状态 */
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        if (!isfinite(ekf->x[i])) {
            memcpy(ekf->x, x_old, sizeof(x_old));
            Quaternion_Normalize(ekf->x);
            EKF_ResetCovariance(ekf);
            break;
        }
    }
}

// ==================================
// 欧拉角
// ==================================
void EKF_GetEuler(const EKF_Handle_t* ekf, float32_t* roll, float32_t* pitch, float32_t* yaw) {
    float32_t q0 = ekf->x[0], q1 = ekf->x[1], q2 = ekf->x[2], q3 = ekf->x[3];

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    *pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

float32_t EKF_GetAltitude(const EKF_Handle_t* ekf) {
    return -ekf->x[EKF_IDX_ALT];
}

float32_t EKF_GetVelocityZ(const EKF_Handle_t* ekf) {
    return -ekf->x[EKF_IDX_VZ];
}
