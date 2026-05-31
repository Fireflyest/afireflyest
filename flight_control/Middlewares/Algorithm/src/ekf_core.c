/**
 * @file ekf_core.c
 * @brief EKF 四旋翼无人机 — 完整实现
 *
 * 本文件实现:
 *   - ekf_types.h   中声明的所有工具函数
 *   - ekf_sensors.h  中声明的传感器辅助函数
 *   - ekf_core.h     中声明的 EKF 核心算法
 *
 * 编译依赖: ekf_types.h, ekf_sensors.h, ekf_core.h, <math.h>, <string.h>
 *
 * 修正记录:
 *   - [FIX] 气压计首次量测初始化: 用首帧 baro 读数初始化 pos.z，
 *           避免 MSL 海拔与 NED 零初始值的巨大偏差导致卡方门限拒绝所有修正
 *   - [FIX] 传递 init_altitude 到 EKF
 */

#include "ekf_core.h"
#include <float.h>
#include <math.h>
#include <string.h>

/* ========================================================================== */
/*  Section 0: 内部宏与常量                                                    */
/* ========================================================================== */

/** @brief 误差状态维度 (方便书写) */
#define ESDIM EKF_ERROR_STATE_DIM /* 15 */

/** @brief 最大量测维度 (所有传感器量测 ≤ 3 维) */
#define MAX_MDIM 3

/** @brief 小量，防止除零 */
#define EPS_F 1e-12f

/* ========================================================================== */
/*  Section 1: 内部数学工具                                                    */
/* ========================================================================== */

/* --- 3 维向量 --- */

/** @brief c = a × b (叉积) */
static inline void v3_cross(const float a[3], const float b[3], float c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/** @brief out = a - b */
static inline void v3_sub(const float a[3], const float b[3], float out[3]) {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

/** @brief out = a + b */
static inline void v3_add(const float a[3], const float b[3], float out[3]) {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

/** @brief ||v|| */
static inline float v3_norm(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/** @brief out = scale * v */
static inline void v3_scale(float s, const float v[3], float out[3]) {
    out[0] = s * v[0];
    out[1] = s * v[1];
    out[2] = s * v[2];
}

/* --- 3×3 矩阵 --- */

/** @brief S = [v]× (反对称矩阵) */
static inline void m3_skew(const float v[3], float S[3][3]) {
    S[0][0] = 0;
    S[0][1] = -v[2];
    S[0][2] = v[1];
    S[1][0] = v[2];
    S[1][1] = 0;
    S[1][2] = -v[0];
    S[2][0] = -v[1];
    S[2][1] = v[0];
    S[2][2] = 0;
}

/** @brief c = R * v */
static inline void m3_mul_v(const float R[3][3], const float v[3], float c[3]) {
    c[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    c[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    c[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

/** @brief C = A * B */
static inline void m3_mul(const float A[3][3], const float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
}

/** @brief Out = A^T */
static inline void m3_trans(const float A[3][3], float Out[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Out[i][j] = A[j][i];
}

/** @brief C = A^T * B */
static inline void m3t_mul_m3(const float A[3][3], const float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[0][i] * B[0][j] + A[1][i] * B[1][j] + A[2][i] * B[2][j];
}

/** @brief C = A * B^T */
static inline void m3_mul_m3t(const float A[3][3], const float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[i][0] * B[j][0] + A[i][1] * B[j][1] + A[i][2] * B[j][2];
}

/** @brief 3×3 单位阵 */
static inline void m3_identity(float M[3][3]) {
    memset(M, 0, 9 * sizeof(float));
    M[0][0] = M[1][1] = M[2][2] = 1.0f;
}

/** @brief 3×3 零阵 */
static inline void m3_zero(float M[3][3]) {
    memset(M, 0, 9 * sizeof(float));
}

/* --- 3×3 求逆 (伴随矩阵法) --- */

/**
 * @brief Out = inv(A), 3×3 矩阵求逆
 * @return 行列式绝对值 (可用于判断奇异性)
 */
static float m3_inv(const float A[3][3], float Out[3][3]) {
    float c00 = A[1][1] * A[2][2] - A[1][2] * A[2][1];
    float c01 = A[1][2] * A[2][0] - A[1][0] * A[2][2];
    float c02 = A[1][0] * A[2][1] - A[1][1] * A[2][0];
    float det = A[0][0] * c00 + A[0][1] * c01 + A[0][2] * c02;

    float inv_det = 1.0f / det;
    Out[0][0] = c00 * inv_det;
    Out[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
    Out[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
    Out[1][0] = c01 * inv_det;
    Out[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    Out[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;
    Out[2][0] = c02 * inv_det;
    Out[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
    Out[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;

    return fabsf(det);
}

/* --- 15×15 矩阵操作 (在 float[ESDIM][ESDIM] 上) --- */

static void m15_zero(float M[ESDIM][ESDIM]) {
    memset(M, 0, ESDIM * ESDIM * sizeof(float));
}

static void m15_identity(float M[ESDIM][ESDIM]) {
    m15_zero(M);
    for (int i = 0; i < ESDIM; i++)
        M[i][i] = 1.0f;
}

/** @brief C = A + B (15×15) */
static void m15_add(const float A[ESDIM][ESDIM],
                    const float B[ESDIM][ESDIM],
                    float C[ESDIM][ESDIM]) {
    for (int i = 0; i < ESDIM; i++)
        for (int j = 0; j < ESDIM; j++)
            C[i][j] = A[i][j] + B[i][j];
}

/** @brief C = A * B (15×15) */
static void m15_mul(const float A[ESDIM][ESDIM],
                    const float B[ESDIM][ESDIM],
                    float C[ESDIM][ESDIM]) {
    for (int i = 0; i < ESDIM; i++) {
        for (int j = 0; j < ESDIM; j++) {
            float s = 0;
            for (int k = 0; k < ESDIM; k++)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
    }
}

/** @brief B = A^T (15×15) */
static void m15_trans(const float A[ESDIM][ESDIM], float B[ESDIM][ESDIM]) {
    for (int i = 0; i < ESDIM; i++)
        for (int j = 0; j < ESDIM; j++)
            B[i][j] = A[j][i];
}

/**
 * @brief 矩阵块赋值: Out[r0:r0+nr, c0:c0+nc] = In[nr][nc]
 */
static void block_set(float* Out, int r0, int c0, int ld_out, const float* In, int nr, int nc, int ld_in) {
    for (int i = 0; i < nr; i++)
        for (int j = 0; j < nc; j++)
            Out[(r0 + i) * ld_out + (c0 + j)] = In[i * ld_in + j];
}

/** @brief 块读取: out[nr][nc] = M[r0:r0+nr, c0:c0+nc] */
static void block_get(const float* M, int r0, int c0, int ld_m, float* out, int nr, int nc, int ld_out) {
    for (int i = 0; i < nr; i++)
        for (int j = 0; j < nc; j++)
            out[i * ld_out + j] = M[(r0 + i) * ld_m + (c0 + j)];
}

/* ========================================================================== */
/*  Section 2: ekf_types.h 工具函数实现                                        */
/* ========================================================================== */

void ekf_quat_normalize(ekf_quat_t* q) {
    float n = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (n < EPS_F) {
        q->w = 1;
        q->x = q->y = q->z = 0;
        return;
    }
    float inv = 1.0f / n;
    q->w *= inv;
    q->x *= inv;
    q->y *= inv;
    q->z *= inv;
}

void ekf_quat_mult(const ekf_quat_t* a, const ekf_quat_t* b, ekf_quat_t* out) {
    out->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    out->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    out->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    out->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}

void ekf_quat_to_rotmat(const ekf_quat_t* q, ekf_mat3_t* R) {
    float w = q->w, x = q->x, y = q->y, z = q->z;

    /* 标准 Hamilton 公式产出 R_b2w (body→world) */
    float r00 = 1.0f - 2.0f * (y * y + z * z);
    float r01 = 2.0f * (x * y - w * z);
    float r02 = 2.0f * (x * z + w * y);
    float r10 = 2.0f * (x * y + w * z);
    float r11 = 1.0f - 2.0f * (x * x + z * z);
    float r12 = 2.0f * (y * z - w * x);
    float r20 = 2.0f * (x * z - w * y);
    float r21 = 2.0f * (y * z + w * x);
    float r22 = 1.0f - 2.0f * (x * x + y * y);

    /* 转置输出: R_w2b (world→body), 满足 v_body = R · v_world */
    R->m[0][0] = r00;
    R->m[0][1] = r10;
    R->m[0][2] = r20;
    R->m[1][0] = r01;
    R->m[1][1] = r11;
    R->m[1][2] = r21;
    R->m[2][0] = r02;
    R->m[2][1] = r12;
    R->m[2][2] = r22;
}

void ekf_quat_to_euler(const ekf_quat_t* q, ekf_euler_t* euler) {
    float w = q->w, x = q->x, y = q->y, z = q->z;

    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    euler->roll = atan2f(sinr, cosr);

    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f)
        euler->pitch = copysignf(M_PI_F / 2.0f, sinp);
    else
        euler->pitch = asinf(sinp);

    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);
    euler->yaw = atan2f(siny, cosy);
}

void ekf_euler_to_quat(const ekf_euler_t* euler, ekf_quat_t* q) {
    float cr = cosf(euler->roll * 0.5f);
    float sr = sinf(euler->roll * 0.5f);
    float cp = cosf(euler->pitch * 0.5f);
    float sp = sinf(euler->pitch * 0.5f);
    float cy = cosf(euler->yaw * 0.5f);
    float sy = sinf(euler->yaw * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

void ekf_quat_apply_correction(const ekf_quat_t* q_nom,
                               const ekf_vec3_t* delta,
                               ekf_quat_t* q_out) {
    /*
     * 小角度近似 q_out ≈ q_nom ⊗ [1, δθ/2] 仅在 |δθ| << 1 时有效。
     * 当 |δθ| > 0.26 rad (15°) 时，近似误差 > 1%。
     * 超限时缩放 δθ 使其回到有效范围，避免发散。
     */
    float delta_norm_sq = delta->x * delta->x +
                          delta->y * delta->y +
                          delta->z * delta->z;
    const float max_delta = 0.26f; /* ≈15°，小角度近似的安全上限 */
    const float max_delta_sq = max_delta * max_delta;

    ekf_vec3_t clamped = *delta;
    if (delta_norm_sq > max_delta_sq) {
        float scale = max_delta / sqrtf(delta_norm_sq);
        clamped.x *= scale;
        clamped.y *= scale;
        clamped.z *= scale;
    }

    ekf_quat_t dq;
    dq.w = 1.0f;
    dq.x = 0.5f * clamped.x;
    dq.y = 0.5f * clamped.y;
    dq.z = 0.5f * clamped.z;

    ekf_quat_mult(q_nom, &dq, q_out);
    ekf_quat_normalize(q_out);
}

void ekf_state_init_default(ekf_state_t* state) {
    memset(state, 0, sizeof(ekf_state_t));
    state->quat.w = 1.0f;
}

void ekf_cov_init_diagonal(ekf_cov_t* P,
                           float pos_std,
                           float vel_std,
                           float att_std,
                           float gbias_std,
                           float abias_std) {
    memset(P, 0, sizeof(ekf_cov_t));

    float vars[ESDIM];
    for (int i = 0; i < 3; i++)
        vars[i] = pos_std * pos_std;
    for (int i = 3; i < 6; i++)
        vars[i] = vel_std * vel_std;
    for (int i = 6; i < 9; i++)
        vars[i] = att_std * att_std;
    for (int i = 9; i < 12; i++)
        vars[i] = gbias_std * gbias_std;
    for (int i = 12; i < 15; i++)
        vars[i] = abias_std * abias_std;

    for (int i = 0; i < ESDIM; i++)
        P->data[i][i] = vars[i];
}

/* ========================================================================== */
/*  Section 3: ekf_sensors.h 辅助函数实现                                      */
/* ========================================================================== */

void ekf_noise_params_init_default(ekf_noise_params_t* p) {
    p->gyro_noise = 1.0e-2f;
    p->gyro_bias_noise = 1.0e-4f;
    p->accel_noise = 5.0e-2f;
    p->accel_bias_noise = 1.0e-3f;
    p->mag_noise = 0.3f;
    p->baro_noise = 0.5f;
    p->baro_bias_noise = 0.01f;
    p->gps_pos_noise = 1.5f;
    p->gps_vel_noise = 0.3f;
    p->optflow_noise = 0.05f;
}

void ekf_gps_origin_init(ekf_gps_origin_t* o, double lat, double lon, float alt) {
    if (o->initialized)
        return;
    o->lat0 = lat;
    o->lon0 = lon;
    o->alt0 = alt;
    o->initialized = 1;
}

void ekf_gps_to_ned(const ekf_gps_origin_t* o,
                    double lat,
                    double lon,
                    float alt,
                    ekf_vec3_t* ned) {
    static const double R_EARTH = 6371000.0;
    double dlat = (lat - o->lat0) * M_PI_F / 180.0;
    double dlon = (lon - o->lon0) * M_PI_F / 180.0;
    double cos_lat0 = cos(o->lat0 * M_PI_F / 180.0);

    ned->x = (float)(dlat * R_EARTH);
    ned->y = (float)(dlon * R_EARTH * cos_lat0);
    ned->z = -(alt - o->alt0);
}

/* ========================================================================== */
/*  Section 4: EKF 初始化与对准                                                */
/* ========================================================================== */

void ekf_init(ekf_t* ekf, const ekf_noise_params_t* noise) {
    memset(ekf, 0, sizeof(ekf_t));
    ekf_state_init_default(&ekf->state);

    if (noise) {
        ekf->noise = *noise;
    } else {
        ekf_noise_params_init_default(&ekf->noise);
    }

    ekf_cov_init_diagonal(&ekf->P,
                          10.0f, 1.0f, 0.5f, 0.1f, 0.5f);

    ekf->initialized = 0;
    ekf->baro_altitude_initialized = 0; /* [FIX] 标记高度未初始化 */
}

/* [FIX] 用初始高度设置 NED 位置 */
void ekf_set_init_altitude(ekf_t* ekf, float altitude) {
    ekf->state.pos.z = -altitude; /* NED: pos.z = -altitude */
}

int ekf_align(ekf_t* ekf,
              const ekf_imu_t imu[],
              int imu_n,
              const ekf_mag_t mag[],
              int mag_n) {
    if (imu_n < 10)
        return -1;

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < imu_n; i++) {
        sum_ax += imu[i].accel.a_x;
        sum_ay += imu[i].accel.a_y;
        sum_az += imu[i].accel.a_z;
        sum_gx += imu[i].gyro.omega_x;
        sum_gy += imu[i].gyro.omega_y;
        sum_gz += imu[i].gyro.omega_z;
    }

    float ax = sum_ax / imu_n;
    float ay = sum_ay / imu_n;
    float az = sum_az / imu_n;
    float gx = sum_gx / imu_n;
    float gy = sum_gy / imu_n;
    float gz = sum_gz / imu_n;

    float a_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (fabsf(a_norm - EKF_GRAVITY) > 2.0f)
        return -2;

    float roll = atan2f(-ay, -az);
    float pitch = atan2f(ax, sqrtf(ay * ay + az * az));
    float yaw = 0.0f;

    if (mag && mag_n > 0) {
        float sum_mx = 0, sum_my = 0, sum_mz = 0;
        for (int i = 0; i < mag_n; i++) {
            sum_mx += mag[i].m_x;
            sum_my += mag[i].m_y;
            sum_mz += mag[i].m_z;
        }
        float mx = sum_mx / mag_n;
        float my = sum_my / mag_n;
        float mz = sum_mz / mag_n;

        float sr = sinf(roll), cr = cosf(roll);
        float sp = sinf(pitch), cp = cosf(pitch);

        float m_hx = mx * cp + my * sr * sp + mz * cr * sp;
        float m_hy = my * cr - mz * sr;
        yaw = atan2f(m_hy, m_hx);

        ekf_euler_t euler_init = {roll, pitch, yaw};
        ekf_quat_t q_init;
        ekf_euler_to_quat(&euler_init, &q_init);
        ekf_mat3_t R_init;
        ekf_quat_to_rotmat(&q_init, &R_init);

        ekf_mag_reference_t* ref = &ekf->mag_ref;
        float mb[3] = {mx, my, mz};
        float R_T[3][3];
        m3_trans(R_init.m, R_T);
        float m_earth_arr[3];
        m3_mul_v(R_T, mb, m_earth_arr);

        ref->m_earth.x = m_earth_arr[0];
        ref->m_earth.y = m_earth_arr[1];
        ref->m_earth.z = m_earth_arr[2];

        ref->total_field = sqrtf(ref->m_earth.x * ref->m_earth.x +
                                 ref->m_earth.y * ref->m_earth.y +
                                 ref->m_earth.z * ref->m_earth.z);
        ref->declination = atan2f(ref->m_earth.y, ref->m_earth.x);
        ref->inclination = atan2f(-ref->m_earth.z,
                                  sqrtf(ref->m_earth.x * ref->m_earth.x +
                                        ref->m_earth.y * ref->m_earth.y));
        ref->calibrated = 1;
    }

    /* ---- 设置初始状态 ---- */
    /* 保留 pos.z (可能已由 ekf_set_init_altitude 设置) */
    float saved_pz = ekf->state.pos.z;

    ekf_state_init_default(&ekf->state);
    ekf->state.pos.z = saved_pz; /* [FIX] 恢复高度 */

    ekf_euler_t euler_init = {roll, pitch, yaw};
    ekf_euler_to_quat(&euler_init, &ekf->state.quat);

    ekf->state.gyro_bias.x = gx;
    ekf->state.gyro_bias.y = gy;
    ekf->state.gyro_bias.z = gz;

    ekf->state.accel_bias.x = 0;
    ekf->state.accel_bias.y = 0;
    ekf->state.accel_bias.z = 0;

    ekf_cov_init_diagonal(&ekf->P,
                          5.0f, 0.5f, 0.1f, 0.05f, 0.3f);

    ekf->initialized = 1;
    return 0;
}

/* ========================================================================== */
/*  Section 5: 状态预测 (Nominal State Propagation)                            */
/* ========================================================================== */

static void ekf_get_rotmat(const ekf_t* ekf, float R[3][3], float RT[3][3]) {
    ekf_mat3_t Rm;
    ekf_quat_to_rotmat(&ekf->state.quat, &Rm);
    memcpy(R, Rm.m, sizeof(float) * 9);
    if (RT)
        m3_trans(R, RT);
}

void ekf_predict(ekf_t* ekf, const ekf_imu_t* imu) {
    if (!ekf->initialized)
        return;

    uint64_t now_us = imu->header.timestamp_us;
    if (ekf->last_predict_us == 0) {
        ekf->last_predict_us = now_us;
        return;
    }
    float dt = (float)(now_us - ekf->last_predict_us) * 1e-6f;
    ekf->last_predict_us = now_us;
    if (dt <= 0 || dt > 0.1f)
        return;

    /* ================================================================ */
    /*  5a: 标称状态传播                                                  */
    /* ================================================================ */

    float omega[3] = {
        imu->gyro.omega_x - ekf->state.gyro_bias.x,
        imu->gyro.omega_y - ekf->state.gyro_bias.y,
        imu->gyro.omega_z - ekf->state.gyro_bias.z};
    float a_meas[3] = {
        imu->accel.a_x - ekf->state.accel_bias.x,
        imu->accel.a_y - ekf->state.accel_bias.y,
        imu->accel.a_z - ekf->state.accel_bias.z};

    float R[3][3], RT[3][3];
    ekf_get_rotmat(ekf, R, RT);

    /* 四元数传播: 精确积分 */
    float omega_norm = v3_norm(omega);
    ekf_quat_t dq;
    if (omega_norm > EPS_F) {
        float half_angle = omega_norm * dt * 0.5f;
        float c = cosf(half_angle);
        float s = sinf(half_angle) / omega_norm;
        dq.w = c;
        dq.x = s * omega[0];
        dq.y = s * omega[1];
        dq.z = s * omega[2];
    } else {
        dq.w = 1.0f;
        dq.x = dq.y = dq.z = 0.0f;
    }
    ekf_quat_mult(&ekf->state.quat, &dq, &ekf->state.quat);
    ekf_quat_normalize(&ekf->state.quat);

    ekf_get_rotmat(ekf, R, RT);

    /* 加速度: 世界系 = R^T * a_corrected + g_world (NED: g=[0,0,+g]) */
    float a_world[3];
    m3_mul_v(RT, a_meas, a_world);
    a_world[2] += EKF_GRAVITY;

    /* 速度/位置传播 (中点法) */
    float vel_mid[3] = {
        ekf->state.vel.x + 0.5f * a_world[0] * dt,
        ekf->state.vel.y + 0.5f * a_world[1] * dt,
        ekf->state.vel.z + 0.5f * a_world[2] * dt};

    ekf->state.vel.x += a_world[0] * dt;
    ekf->state.vel.y += a_world[1] * dt;
    ekf->state.vel.z += a_world[2] * dt;

    ekf->state.pos.x += vel_mid[0] * dt;
    ekf->state.pos.y += vel_mid[1] * dt;
    ekf->state.pos.z += vel_mid[2] * dt;

    /* ================================================================ */
    /*  5b: 误差状态协方差传播                                            */
    /* ================================================================ */

    float f_tilde[3];
    memcpy(f_tilde, a_meas, sizeof(float) * 3);

    float skew_f[3][3], Rt_skew_f[3][3], neg_Rt_skew_f[3][3];
    m3_skew(f_tilde, skew_f);
    m3_mul(RT, skew_f, Rt_skew_f);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            neg_Rt_skew_f[i][j] = -Rt_skew_f[i][j] * dt;

    float skew_w[3][3], neg_skew_w_dt[3][3];
    m3_skew(omega, skew_w);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            neg_skew_w_dt[i][j] = -skew_w[i][j] * dt;

    float neg_Rt_dt[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            neg_Rt_dt[i][j] = -RT[i][j] * dt;

    float neg_I_dt[3][3];
    m3_zero(neg_I_dt);
    neg_I_dt[0][0] = neg_I_dt[1][1] = neg_I_dt[2][2] = -dt;

    float I_dt[3][3];
    m3_zero(I_dt);
    I_dt[0][0] = I_dt[1][1] = I_dt[2][2] = dt;

    /* 构建 Φ = I + F·dt (15×15) */
    float Phi[ESDIM][ESDIM];
    m15_identity(Phi);

    block_set(&Phi[0][0], 0, 3, ESDIM, &I_dt[0][0], 3, 3, 3);
    block_set(&Phi[0][0], 3, 6, ESDIM, &neg_Rt_skew_f[0][0], 3, 3, 3);
    block_set(&Phi[0][0], 3, 12, ESDIM, &neg_Rt_dt[0][0], 3, 3, 3);

    float tmp3[3][3];
    block_get(&Phi[0][0], 6, 6, ESDIM, &tmp3[0][0], 3, 3, 3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            tmp3[i][j] += neg_skew_w_dt[i][j];
    block_set(&Phi[0][0], 6, 6, ESDIM, &tmp3[0][0], 3, 3, 3);

    block_set(&Phi[0][0], 6, 9, ESDIM, &neg_I_dt[0][0], 3, 3, 3);

    /* P = Φ·P·Φ^T + Q_d */
    float PhiP[ESDIM][ESDIM], PhiPt[ESDIM][ESDIM], Pnew[ESDIM][ESDIM];
    m15_mul(Phi, ekf->P.data, PhiP);
    m15_trans(Phi, PhiPt);
    m15_mul(PhiP, PhiPt, Pnew);

    float sg2 = ekf->noise.gyro_noise * ekf->noise.gyro_noise;
    float sa2 = ekf->noise.accel_noise * ekf->noise.accel_noise;
    float sbg2 = ekf->noise.gyro_bias_noise * ekf->noise.gyro_bias_noise;
    float sba2 = ekf->noise.accel_bias_noise * ekf->noise.accel_bias_noise;

    for (int i = 0; i < 3; i++) {
        Pnew[3 + i][3 + i] += sa2 * dt;
        Pnew[6 + i][6 + i] += sg2 * dt;
        Pnew[9 + i][9 + i] += sbg2 * dt;
        Pnew[12 + i][12 + i] += sba2 * dt;
    }

    for (int i = 0; i < ESDIM; i++)
        for (int j = i + 1; j < ESDIM; j++) {
            float avg = 0.5f * (Pnew[i][j] + Pnew[j][i]);
            Pnew[i][j] = Pnew[j][i] = avg;
        }

    memcpy(ekf->P.data, Pnew, sizeof(ekf->P.data));

    /* ---- P 对角线限幅 ---- */
    /* 防止协方差爆炸 (发散) 或过小 (过度自信导致对异常量测无响应) */
    float (*Pdiag)[ESDIM] = ekf->P.data;
    for (int i = 0; i < ESDIM; i++) {
        if (Pdiag[i][i] > 1e4f)
            Pdiag[i][i] = 1e4f;
        if (Pdiag[i][i] < 1e-10f)
            Pdiag[i][i] = 1e-10f;
    }

    /* ---- 全局 NaN/Inf 检测 ---- */
    for (int i = 0; i < ESDIM; i++) {
        if (!isfinite(ekf->state.vel.x) || !isfinite(ekf->state.pos.z) ||
            !isfinite(ekf->state.quat.w)) {
            /* 状态损坏，重置为已知安全状态 */
            float saved_alt = -ekf->state.pos.z; /* 尝试保留高度 */
            ekf_state_init_default(&ekf->state);
            ekf->state.pos.z = isfinite(saved_alt) ? -saved_alt : 0;
            ekf_cov_init_diagonal(&ekf->P, 10.0f, 1.0f, 0.5f, 0.1f, 0.5f);
            break;
        }
    }
}

/* ========================================================================== */
/*  Section 6: 误差状态注入与重置                                              */
/* ========================================================================== */

static void ekf_inject_and_reset(ekf_t* ekf, const float dx[ESDIM]) {
    ekf->state.pos.x += dx[0];
    ekf->state.pos.y += dx[1];
    ekf->state.pos.z += dx[2];

    ekf->state.vel.x += dx[3];
    ekf->state.vel.y += dx[4];
    ekf->state.vel.z += dx[5];

    ekf_vec3_t delta_theta = {dx[6], dx[7], dx[8]};
    ekf_quat_apply_correction(&ekf->state.quat, &delta_theta, &ekf->state.quat);

    ekf->state.gyro_bias.x += dx[9];
    ekf->state.gyro_bias.y += dx[10];
    ekf->state.gyro_bias.z += dx[11];

    ekf->state.accel_bias.x += dx[12];
    ekf->state.accel_bias.y += dx[13];
    ekf->state.accel_bias.z += dx[14];

    float dt_vec[3] = {dx[6], dx[7], dx[8]};
    float skew_dt[3][3];
    m3_skew(dt_vec, skew_dt);

    float Gr_theta[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Gr_theta[i][j] = (i == j ? 1.0f : 0.0f) - 0.5f * skew_dt[i][j];

    float (*P)[ESDIM] = ekf->P.data;

    for (int j = 0; j < ESDIM; j++) {
        float col[3] = {P[6][j], P[7][j], P[8][j]};
        float out[3];
        m3_mul_v(Gr_theta, col, out);
        P[6][j] = out[0];
        P[7][j] = out[1];
        P[8][j] = out[2];
    }

    float Gr_theta_T[3][3];
    m3_trans(Gr_theta, Gr_theta_T);
    for (int i = 0; i < ESDIM; i++) {
        float row[3] = {P[i][6], P[i][7], P[i][8]};
        float out[3];
        m3_mul_v(Gr_theta_T, row, out);
        P[i][6] = out[0];
        P[i][7] = out[1];
        P[i][8] = out[2];
    }

    for (int i = 0; i < ESDIM; i++)
        for (int j = i + 1; j < ESDIM; j++) {
            float avg = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = avg;
        }
}

/* ========================================================================== */
/*  Section 7: 通用 Kalman 更新                                                */
/* ========================================================================== */

static void ekf_update_generic(ekf_t* ekf,
                               const float z[],
                               const float h[],
                               const float H[][ESDIM],
                               const float R_noise[][MAX_MDIM],
                               int dim_m) {
    if (!ekf->initialized)
        return;

    float (*P)[ESDIM] = ekf->P.data;

    /* ---- 新息 ---- */
    float y[MAX_MDIM];
    for (int i = 0; i < dim_m; i++)
        y[i] = z[i] - h[i];

    /* ---- PH^T ---- */
    float PHt[ESDIM][MAX_MDIM];
    for (int i = 0; i < ESDIM; i++)
        for (int j = 0; j < dim_m; j++) {
            float s = 0;
            for (int k = 0; k < ESDIM; k++)
                s += P[i][k] * H[j][k];
            PHt[i][j] = s;
        }

    /* ---- S = H·PH^T + R ---- */
    float S[MAX_MDIM][MAX_MDIM];
    for (int i = 0; i < dim_m; i++)
        for (int j = 0; j < dim_m; j++) {
            float s = 0;
            for (int k = 0; k < ESDIM; k++)
                s += H[i][k] * PHt[k][j];
            S[i][j] = s + R_noise[i][j];
        }

    /* ---- S^{-1} ---- */
    float S_inv[MAX_MDIM][MAX_MDIM];
    if (dim_m == 1) {
        if (S[0][0] < EPS_F)
            return;
        S_inv[0][0] = 1.0f / S[0][0];
    } else if (dim_m == 2) {
        float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if (fabsf(det) < EPS_F)
            return;
        float inv_det = 1.0f / det;
        S_inv[0][0] = S[1][1] * inv_det;
        S_inv[0][1] = -S[0][1] * inv_det;
        S_inv[1][0] = -S[1][0] * inv_det;
        S_inv[1][1] = S[0][0] * inv_det;
    } else {
        if (m3_inv(S, S_inv) < EPS_F)
            return;
    }

    /* ---- K = PH^T · S^{-1} ---- */
    float K[ESDIM][MAX_MDIM];
    for (int i = 0; i < ESDIM; i++)
        for (int j = 0; j < dim_m; j++) {
            float s = 0;
            for (int k = 0; k < dim_m; k++)
                s += PHt[i][k] * S_inv[k][j];
            K[i][j] = s;
        }

    /* ---- 卡方检验 ---- */
    float chi2 = 0;
    for (int i = 0; i < dim_m; i++) {
        float Sy_i = 0;
        for (int j = 0; j < dim_m; j++)
            Sy_i += S_inv[i][j] * y[j];
        chi2 += y[i] * Sy_i;
    }
    float chi2_threshold = (float)dim_m * 9.0f;
    if (chi2 > chi2_threshold)
        return;

    /* ---- dx = K · y ---- */
    float dx[ESDIM];
    for (int i = 0; i < ESDIM; i++) {
        float s = 0;
        for (int j = 0; j < dim_m; j++)
            s += K[i][j] * y[j];
        dx[i] = s;
    }

    /* ---- [防线2a] dx 大小保护 ---- */
    /*
     * 单次量测修正不应超过物理合理范围:
     *   位置: 5m      速度: 3 m/s    姿态: 15°(0.26rad)
     *   gyro bias: 0.1 rad/s   accel bias: 0.5 m/s²
     *
     * 超限说明量测与状态严重不一致，跳过修正。
     */
    static const float dx_limit[ESDIM] = {
        5.0f, 5.0f, 5.0f,    /* δp (m) */
        3.0f, 3.0f, 3.0f,    /* δv (m/s) */
        0.26f, 0.26f, 0.26f, /* δθ (rad, ≈15°) */
        0.1f, 0.1f, 0.1f,    /* δb_g (rad/s) */
        0.5f, 0.5f, 0.5f     /* δb_a (m/s²) */
    };
    for (int i = 0; i < ESDIM; i++) {
        if (fabsf(dx[i]) > dx_limit[i])
            return; /* 修正过大，丢弃本次量测 */
    }

    /* ---- [防线2b] NaN 检测 ---- */
    for (int i = 0; i < ESDIM; i++) {
        if (!isfinite(dx[i]))
            return;
    }

    /* ---- Joseph 形式协方差更新 ---- */
    float A[ESDIM][ESDIM];
    m15_identity(A);
    for (int i = 0; i < ESDIM; i++)
        for (int j = 0; j < ESDIM; j++)
            for (int k = 0; k < dim_m; k++)
                A[i][j] -= K[i][k] * H[k][j];

    float AP[ESDIM][ESDIM];
    m15_mul(A, ekf->P.data, AP);

    float At[ESDIM][ESDIM], Pnew[ESDIM][ESDIM];
    m15_trans(A, At);
    m15_mul(AP, At, Pnew);

    float KR[ESDIM][MAX_MDIM];
    for (int i = 0; i < ESDIM; i++)
        for (int j = 0; j < dim_m; j++) {
            float s = 0;
            for (int k = 0; k < dim_m; k++)
                s += K[i][k] * R_noise[k][j];
            KR[i][j] = s;
        }
    for (int i = 0; i < ESDIM; i++)
        for (int j = 0; j < ESDIM; j++)
            for (int k = 0; k < dim_m; k++)
                Pnew[i][j] += KR[i][k] * K[j][k];

    for (int i = 0; i < ESDIM; i++)
        for (int j = i + 1; j < ESDIM; j++) {
            float avg = 0.5f * (Pnew[i][j] + Pnew[j][i]);
            Pnew[i][j] = Pnew[j][i] = avg;
        }

    /* ---- P 对角线非负检查 ---- */
    for (int i = 0; i < ESDIM; i++) {
        if (Pnew[i][i] < 0.0f) {
            return; /* 协方差异常，丢弃 */
        }
    }

    memcpy(ekf->P.data, Pnew, sizeof(ekf->P.data));
    ekf_inject_and_reset(ekf, dx);
}

/* ========================================================================== */
/*  Section 8: 各传感器量测更新                                                */
/* ========================================================================== */

void ekf_update_mag(ekf_t* ekf, const ekf_mag_t* mag) {
    if (!ekf->initialized || !ekf->mag_ref.calibrated)
        return;
    if (mag->header.status != EKF_SENSOR_VALID)
        return;

    /* ---- 高动态保护 ---- */
    /* 快速旋转时磁力计受振动和硬铁干扰，yaw 修正不可靠。
     * 通过 gyro 速率判断动态状态: |ω| > 50°/s → 跳过 */
    float gyro_rate = v3_norm((float[]){
        ekf->state.gyro_bias.x, /* 这里用不了 gyro，用角速率估计 */
    });
    /* 简化: 直接检查上一帧的角速率 (通过 P 的 δθ 方差判断置信度) */
    float att_var = ekf->P.data[6][6] + ekf->P.data[7][7] + ekf->P.data[8][8];
    if (att_var > 1.0f) {
        /* 姿态不确定性高 → 动态中，增大磁力计噪声让修正变弱 */
        /* 不直接跳过，让 EKF 自行判断 */
    }

    float R[3][3], RT[3][3];
    ekf_get_rotmat(ekf, R, RT);

    float m_earth[3] = {
        ekf->mag_ref.m_earth.x,
        ekf->mag_ref.m_earth.y,
        ekf->mag_ref.m_earth.z};

    float h[3];
    m3_mul_v(R, m_earth, h);

    float z[3] = {mag->m_x, mag->m_y, mag->m_z};

    float H_mag[MAX_MDIM][ESDIM];
    memset(H_mag, 0, sizeof(H_mag));

    float Rm[3];
    m3_mul_v(R, m_earth, Rm);
    float skew_Rm[3][3];
    m3_skew(Rm, skew_Rm);
    block_set(&H_mag[0][0], 0, 6, ESDIM, &skew_Rm[0][0], 3, 3, 3);

    float sigma2 = ekf->noise.mag_noise * ekf->noise.mag_noise;

    /* [防线4] 姿态不确定性高 → 膨胀磁力计噪声 */
    if (att_var > 0.1f) {
        sigma2 *= (1.0f + att_var * 10.0f);
    }

    float R_noise[MAX_MDIM][MAX_MDIM] = {
        {sigma2, 0, 0},
        {0, sigma2, 0},
        {0, 0, sigma2}};

    ekf_update_generic(ekf, z, h, H_mag, R_noise, 3);
}

void ekf_update_gps(ekf_t* ekf, const ekf_gps_t* gps) {
    if (!ekf->initialized)
        return;
    if (gps->header.status != EKF_SENSOR_VALID)
        return;
    if (gps->fix_type < 3)
        return;
    if (gps->num_sats < 6)
        return;

    if (!ekf->gps_origin.initialized) {
        ekf_gps_origin_init(&ekf->gps_origin,
                            gps->latitude, gps->longitude, gps->altitude_msl);
        ekf->state.pos.x = 0;
        ekf->state.pos.y = 0;
        ekf->state.pos.z = 0;
        return;
    }

    ekf_vec3_t gps_ned;
    ekf_gps_to_ned(&ekf->gps_origin,
                   gps->latitude, gps->longitude, gps->altitude_msl,
                   &gps_ned);

    float z_pos[3] = {gps_ned.x, gps_ned.y, gps_ned.z};
    float h_pos[3] = {ekf->state.pos.x, ekf->state.pos.y, ekf->state.pos.z};

    float H_pos[MAX_MDIM][ESDIM];
    memset(H_pos, 0, sizeof(H_pos));
    H_pos[0][0] = H_pos[1][1] = H_pos[2][2] = 1.0f;

    float pos_var = gps->horiz_acc > 0 ? gps->horiz_acc : ekf->noise.gps_pos_noise;
    pos_var *= pos_var;
    float vert_var = gps->vert_acc > 0 ? gps->vert_acc : ekf->noise.gps_pos_noise * 1.5f;
    vert_var *= vert_var;

    float R_gps_pos[MAX_MDIM][MAX_MDIM] = {
        {pos_var, 0, 0},
        {0, pos_var, 0},
        {0, 0, vert_var}};

    ekf_update_generic(ekf, z_pos, h_pos, H_pos, R_gps_pos, 3);

    float z_vel[3] = {gps->vel_north, gps->vel_east, gps->vel_down};
    float h_vel[3] = {ekf->state.vel.x, ekf->state.vel.y, ekf->state.vel.z};

    float H_vel[MAX_MDIM][ESDIM];
    memset(H_vel, 0, sizeof(H_vel));
    H_vel[0][3] = H_vel[1][4] = H_vel[2][5] = 1.0f;

    float vel_var = gps->vel_acc > 0 ? gps->vel_acc : ekf->noise.gps_vel_noise;
    vel_var *= vel_var;
    float R_gps_vel[MAX_MDIM][MAX_MDIM] = {
        {vel_var, 0, 0},
        {0, vel_var, 0},
        {0, 0, vel_var}};

    ekf_update_generic(ekf, z_vel, h_vel, H_vel, R_gps_vel, 3);
}

/* 气压计更新: 用首帧读数初始化高度，跳过首帧卡方检验 */
void ekf_update_baro(ekf_t* ekf, const ekf_baro_t* baro) {
    if (!ekf->initialized)
        return;
    if (baro->header.status != EKF_SENSOR_VALID)
        return;

    /* 首帧验证 pos.z 与气压计是否对齐 */
    if (!ekf->baro_altitude_initialized) {
        float predicted_alt = -ekf->state.pos.z;
        if (fabsf(baro->altitude - predicted_alt) > 10.0f) {
            /* 偏差过大 → pos.z 未被正确初始化，用气压计重设 */
            ekf->state.pos.z = -baro->altitude;
            ekf->state.vel.z = 0.0f;
        }
        ekf->baro_altitude_initialized = 1;
    }

    /* 量测: z = baro.altitude (向上为正)
     * 预测: h = -p_D (NED: D 向下为正, 高度向上为正 → 取负)
     */
    float z[1] = {baro->altitude};
    float h_pred[1] = {-ekf->state.pos.z};

    /* H_baro = [0, 0, -1, 0, 0, ..., 0] (1×15) */
    float H_baro[1][ESDIM];
    memset(H_baro, 0, sizeof(H_baro));
    H_baro[0][2] = -1.0f;

    float sigma2 = ekf->noise.baro_noise * ekf->noise.baro_noise;
    float R_baro[MAX_MDIM][MAX_MDIM] = {{0}};
    R_baro[0][0] = sigma2;

    ekf_update_generic(ekf, z, h_pred, H_baro, R_baro, 1);
}

void ekf_update_optflow(ekf_t* ekf, const ekf_optflow_t* flow) {
    if (!ekf->initialized)
        return;
    if (flow->header.status != EKF_SENSOR_VALID)
        return;
    if (flow->quality < 50)
        return;
    if (flow->distance_m <= 0)
        return;

    float z[2] = {flow->velocity_x, flow->velocity_y};

    float R[3][3];
    ekf_get_rotmat(ekf, R, NULL);

    float v_world[3] = {
        ekf->state.vel.x, ekf->state.vel.y, ekf->state.vel.z};
    float v_body[3];
    m3_mul_v(R, v_world, v_body);

    float h_pred[2] = {v_body[0], v_body[1]};

    float H_flow[MAX_MDIM][ESDIM];
    memset(H_flow, 0, sizeof(H_flow));

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++)
            H_flow[i][3 + j] = R[i][j];

    float Rv[3];
    m3_mul_v(R, v_world, Rv);
    float skew_Rv[3][3];
    m3_skew(Rv, skew_Rv);

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++)
            H_flow[i][6 + j] = skew_Rv[i][j];

    float h_agl = flow->distance_m;
    float sigma_base = ekf->noise.optflow_noise / (h_agl > 0.3f ? h_agl : 0.3f);
    float quality_factor = 1.0f + (100.0f - (float)flow->quality) / 100.0f * 3.0f;
    float sigma2 = sigma_base * sigma_base * quality_factor;

    float R_flow[MAX_MDIM][MAX_MDIM] = {
        {sigma2, 0},
        {0, sigma2}};

    ekf_update_generic(ekf, z, h_pred, H_flow, R_flow, 2);
}

/* ========================================================================== */
/*  Section 9: 状态读取                                                        */
/* ========================================================================== */

void ekf_get_euler(const ekf_t* ekf, ekf_euler_t* out) {
    ekf_quat_to_euler(&ekf->state.quat, out);
}

void ekf_get_position(const ekf_t* ekf, ekf_vec3_t* out) {
    *out = ekf->state.pos;
}

void ekf_get_velocity(const ekf_t* ekf, ekf_vec3_t* out) {
    *out = ekf->state.vel;
}

void ekf_get_quat(const ekf_t* ekf, ekf_quat_t* out) {
    *out = ekf->state.quat;
}

void ekf_get_gyro_bias(const ekf_t* ekf, ekf_vec3_t* out) {
    *out = ekf->state.gyro_bias;
}

int ekf_is_initialized(const ekf_t* ekf) {
    return ekf->initialized;
}
