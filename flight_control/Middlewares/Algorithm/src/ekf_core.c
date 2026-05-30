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
 * @param Out   目标矩阵
 * @param r0    起始行
 * @param c0    起始列
 * @param In    源矩阵
 * @param nr    行数
 * @param nc    列数
 * @param ld_out Out 的 leading dimension
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
    float w2 = 2 * w, x2 = 2 * x, y2 = 2 * y;
    float ww = w2 * w, xx = x2 * x, yy = y2 * y, zz = 2 * z * z;
    float xy = x2 * y, xz = x2 * z, yz = y2 * z;
    float wx = w2 * x, wy = w2 * y, wz = w2 * z;

    R->m[0][0] = 1 - yy - zz;
    R->m[0][1] = xy - wz;
    R->m[0][2] = xz + wy;
    R->m[1][0] = xy + wz;
    R->m[1][1] = 1 - xx - zz;
    R->m[1][2] = yz - wx;
    R->m[2][0] = xz - wy;
    R->m[2][1] = yz + wx;
    R->m[2][2] = 1 - xx - yy;
}

void ekf_quat_to_euler(const ekf_quat_t* q, ekf_euler_t* euler) {
    /* ZYX 顺序: roll(X), pitch(Y), yaw(Z) */
    float w = q->w, x = q->x, y = q->y, z = q->z;

    /* Roll (X 轴) */
    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    euler->roll = atan2f(sinr, cosr);

    /* Pitch (Y 轴) — 钳位到 [-π/2, π/2] */
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f)
        euler->pitch = copysignf(M_PI_F / 2.0f, sinp);
    else
        euler->pitch = asinf(sinp);

    /* Yaw (Z 轴) */
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
    /* q_out = q_nom ⊗ [1, δθ/2], 然后归一化 */
    float half = 0.5f;
    ekf_quat_t dq;
    dq.w = 1.0f;
    dq.x = half * delta->x;
    dq.y = half * delta->y;
    dq.z = half * delta->z;

    ekf_quat_mult(q_nom, &dq, q_out);
    ekf_quat_normalize(q_out);
}

void ekf_state_init_default(ekf_state_t* state) {
    memset(state, 0, sizeof(ekf_state_t));
    state->quat.w = 1.0f; /* 单位四元数: 水平朝北 */
}

void ekf_cov_init_diagonal(ekf_cov_t* P,
                           float pos_std,
                           float vel_std,
                           float att_std,
                           float gbias_std,
                           float abias_std) {
    memset(P, 0, sizeof(ekf_cov_t));

    /* 误差状态索引: [0-2]=δp, [3-5]=δv, [6-8]=δθ, [9-11]=δb_g, [12-14]=δb_a */
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
    /* 典型小型四旋翼传感器参数 (需根据实际硬件校准) */
    p->gyro_noise = 1.0e-2f;       /* rad/s / √Hz     */
    p->gyro_bias_noise = 1.0e-4f;  /* rad/s / √s      */
    p->accel_noise = 5.0e-2f;      /* m/s² / √Hz      */
    p->accel_bias_noise = 1.0e-3f; /* m/s² / √s       */
    p->mag_noise = 0.3f;           /* μT / √Hz        */
    p->baro_noise = 0.5f;          /* m / √Hz         */
    p->baro_bias_noise = 0.01f;    /* m / √s          */
    p->gps_pos_noise = 1.5f;       /* m (1σ)          */
    p->gps_vel_noise = 0.3f;       /* m/s (1σ)        */
    p->optflow_noise = 0.05f;      /* rad/s / √Hz     */
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
    /* 小角度近似: 距原点 <10km 误差 <1m */
    static const double R_EARTH = 6371000.0;
    double dlat = (lat - o->lat0) * M_PI / 180.0;
    double dlon = (lon - o->lon0) * M_PI / 180.0;
    double cos_lat0 = cos(o->lat0 * M_PI / 180.0);

    ned->x = (float)(dlat * R_EARTH);            /* North */
    ned->y = (float)(dlon * R_EARTH * cos_lat0); /* East  */
    ned->z = -(alt - o->alt0);                   /* Down (高度取反) */
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

    /* 初始协方差 — 较大的不确定性 */
    ekf_cov_init_diagonal(&ekf->P,
                          10.0f, /* 位置: 10 m        */
                          1.0f,  /* 速度: 1 m/s       */
                          0.5f,  /* 姿态: 0.5 rad (~30°) */
                          0.1f,  /* gyro bias: 0.1 rad/s */
                          0.5f); /* accel bias: 0.5 m/s² */

    ekf->initialized = 0;
}

int ekf_align(ekf_t* ekf,
              const ekf_imu_t imu[],
              int imu_n,
              const ekf_mag_t mag[],
              int mag_n) {
    if (imu_n < 10)
        return -1;

    /* ---- Step 1: 计算 IMU 均值 (bias 估计 + 姿态初始化) ---- */
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

    /* 验证加速度幅值 ≈ g */
    float a_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (fabsf(a_norm - EKF_GRAVITY) > 2.0f)
        return -2;

    /* ---- Step 2: 由加速度计估计 roll/pitch ---- */
    /* 加速度计读数 = R * [0,0,-g], 故:
     *   roll  = atan2(-ay, -az)   (注意取负)
     *   pitch = atan2(ax, sqrt(ay²+az²))
     * (FRD 约定, 静止时 [ax,ay,az] ≈ [0,0,-g])
     */
    float roll = atan2f(ay, -az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float yaw = 0.0f;

    /* ---- Step 3: 由磁力计估计 yaw ---- */
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

        /* 将磁力计投影到水平面:
         *   m_N_body = mx*cos(pitch) + my*sin(roll)*sin(pitch) + mz*cos(roll)*sin(pitch)
         *   m_E_body = my*cos(roll) - mz*sin(roll)
         *   yaw = atan2(-m_E_body, m_N_body)
         */
        float sr = sinf(roll), cr = cosf(roll);
        float sp = sinf(pitch), cp = cosf(pitch);

        float m_hx = mx * cp + my * sr * sp + mz * cr * sp;
        float m_hy = my * cr - mz * sr;

        yaw = atan2f(m_hy, m_hx);

        /* 存储地磁参考: 将 body 系磁力计旋转到 NED (使用初始姿态) */
        ekf_euler_t euler_init = {roll, pitch, yaw};
        ekf_quat_t q_init;
        ekf_euler_to_quat(&euler_init, &q_init);
        ekf_mat3_t R_init;
        ekf_quat_to_rotmat(&q_init, &R_init);
        /* m_earth = R^T * m_body (旋转到世界系) */
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

    /* ---- Step 4: 设置初始状态 ---- */
    ekf_state_init_default(&ekf->state);

    /* 姿态 */
    ekf_euler_t euler_init = {roll, pitch, yaw};
    ekf_euler_to_quat(&euler_init, &ekf->state.quat);

    /* Gyro bias: 静止时角速度均值即 bias */
    ekf->state.gyro_bias.x = gx;
    ekf->state.gyro_bias.y = gy;
    ekf->state.gyro_bias.z = gz;

    /* Accel bias: 初始设为 0 (后续 EKF 在线估计) */
    ekf->state.accel_bias.x = 0;
    ekf->state.accel_bias.y = 0;
    ekf->state.accel_bias.z = 0;

    /* 初始协方差: 对准后缩小 */
    ekf_cov_init_diagonal(&ekf->P,
                          5.0f,  /* 位置: 5 m         */
                          0.5f,  /* 速度: 0.5 m/s     */
                          0.1f,  /* 姿态: 0.1 rad (~6°) */
                          0.05f, /* gyro bias: 0.05 rad/s */
                          0.3f); /* accel bias: 0.3 m/s² */

    ekf->initialized = 1;
    return 0;
}

/* ========================================================================== */
/*  Section 5: 状态预测 (Nominal State Propagation)                            */
/* ========================================================================== */

/**
 * @brief 计算旋转矩阵 R (world→body) 和 R^T (body→world)
 */
static void ekf_get_rotmat(const ekf_t* ekf, float R[3][3], float RT[3][3]) {
    ekf_mat3_t Rm;
    ekf_quat_to_rotmat(&ekf->state.quat, &Rm);
    memcpy(R, Rm.m, sizeof(float) * 9);
    m3_trans(R, RT);
}

void ekf_predict(ekf_t* ekf, const ekf_imu_t* imu) {
    if (!ekf->initialized)
        return;

    /* ---- 时间步长 ---- */
    uint64_t now_us = imu->header.timestamp_us;
    if (ekf->last_predict_us == 0) {
        ekf->last_predict_us = now_us;
        return; /* 第一帧只记录时间 */
    }
    float dt = (float)(now_us - ekf->last_predict_us) * 1e-6f;
    ekf->last_predict_us = now_us;
    if (dt <= 0 || dt > 0.1f)
        return; /* 时间异常保护 */

    /* ================================================================ */
    /*  5a: 标称状态传播                                                  */
    /* ================================================================ */

    /* 提取 bias 校正后的 IMU 测量 */
    float omega[3] = {
        imu->gyro.omega_x - ekf->state.gyro_bias.x,
        imu->gyro.omega_y - ekf->state.gyro_bias.y,
        imu->gyro.omega_z - ekf->state.gyro_bias.z};
    float a_meas[3] = {
        imu->accel.a_x - ekf->state.accel_bias.x,
        imu->accel.a_y - ekf->state.accel_bias.y,
        imu->accel.a_z - ekf->state.accel_bias.z};

    /* 旋转矩阵 */
    float R[3][3], RT[3][3];
    ekf_get_rotmat(ekf, R, RT);

    /* (a) 四元数传播: 精确积分
     *   q(k+1) = q(k) ⊗ exp(ω·dt/2)
     *   exp(ω·dt/2) = [cos(θ/2), (ω/|ω|)·sin(θ/2)]
     *   其中 θ = |ω|·dt
     */
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

    /* 更新旋转矩阵 (四元数已更新) */
    ekf_get_rotmat(ekf, R, RT);

    /* (b) 加速度: 世界系 = R^T * a_corrected + g_world
     *   g_world = [0, 0, +g] (NED, 重力指向 +Z_D)
     */
    float a_world[3];
    m3_mul_v(RT, a_meas, a_world);
    a_world[2] += EKF_GRAVITY; /* 加上 NED 系重力 */

    /* (c) 速度/位置传播 (中点法) */
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

    /* bias 模型: 随机游走, 标称传播中不变 */

    /* ================================================================ */
    /*  5b: 误差状态协方差传播                                            */
    /* ================================================================ */
    /*
     * 误差状态: δx = [δp(3), δv(3), δθ(3), δb_g(3), δb_a(3)]
     *
     * 线性化矩阵 F (15×15):
     *
     *     [ 0    I    -R^T*[f̃]×       0        -R^T  ]    δp
     *     [ 0    0    -[ω̃]×           -I        0    ]    δv  (注: δθ 这里是 δv 方程中的)
     * F = [ 0    0     0               0         0    ]    δθ
     *     [ 0    0     0               0         0    ]    δb_g
     *     [ 0    0     0               0         0    ]    δb_a
     *
     * 修正: 正确的 F 矩阵:
     *
     *     [ 0    I     0               0         0    ]    ∂δp/∂δv = I
     *     [ 0    0    -R^T*[f̃]×       0        -R^T  ]    ∂δv/∂δθ, ∂δv/∂δb_a
     * F = [ 0    0    -[ω̃]×          -I         0    ]    ∂δθ/∂δθ, ∂δθ/∂δb_g
     *     [ 0    0     0               0         0    ]    δb_g 随机游走
     *     [ 0    0     0               0         0    ]    δb_a 随机游走
     *
     * 其中:
     *   f̃ = a_meas (bias 校正后的比力, 机体系)
     *   ω̃ = omega  (bias 校正后的角速度, 机体系)
     *
     * 离散传播:
     *   Φ = I + F·dt
     *   P = Φ·P·Φ^T + Q_d
     *
     * Q_d = diag(0, σ²_a·dt·I, σ²_g·dt·I, σ²_bg·dt·I, σ²_ba·dt·I)
     */

    /* 构建 F·dt 的非零块 */
    float f_tilde[3]; /* bias 校正后的比力 */
    memcpy(f_tilde, a_meas, sizeof(float) * 3);

    /* -R^T * [f̃]× */
    float skew_f[3][3], Rt_skew_f[3][3], neg_Rt_skew_f[3][3];
    m3_skew(f_tilde, skew_f);
    m3_mul(RT, skew_f, Rt_skew_f);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            neg_Rt_skew_f[i][j] = -Rt_skew_f[i][j] * dt;

    /* -[ω̃]× · dt */
    float skew_w[3][3], neg_skew_w_dt[3][3];
    m3_skew(omega, skew_w);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            neg_skew_w_dt[i][j] = -skew_w[i][j] * dt;

    /* -R^T · dt (用于 ∂δv/∂δb_a) */
    float neg_Rt_dt[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            neg_Rt_dt[i][j] = -RT[i][j] * dt;

    /* -I · dt (用于 ∂δθ/∂δb_g) */
    float neg_I_dt[3][3];
    m3_zero(neg_I_dt);
    neg_I_dt[0][0] = neg_I_dt[1][1] = neg_I_dt[2][2] = -dt;

    /* I · dt (用于 ∂δp/∂δv) */
    float I_dt[3][3];
    m3_zero(I_dt);
    I_dt[0][0] = I_dt[1][1] = I_dt[2][2] = dt;

    /* 构建 Φ = I + F·dt (15×15) */
    float Phi[ESDIM][ESDIM];
    m15_identity(Phi);

    /* Φ[0:3, 3:6] = I·dt (∂δp/∂δv) */
    block_set(&Phi[0][0], 0, 3, ESDIM, &I_dt[0][0], 3, 3, 3);

    /* Φ[3:6, 6:9] = -R^T·[f̃]×·dt (∂δv/∂δθ) */
    block_set(&Phi[0][0], 3, 6, ESDIM, &neg_Rt_skew_f[0][0], 3, 3, 3);

    /* Φ[3:6, 12:15] = -R^T·dt (∂δv/∂δb_a) */
    block_set(&Phi[0][0], 3, 12, ESDIM, &neg_Rt_dt[0][0], 3, 3, 3);

    /* Φ[6:9, 6:9] += -[ω̃]×·dt (∂δθ/∂δθ) */
    float tmp3[3][3];
    block_get(&Phi[0][0], 6, 6, ESDIM, &tmp3[0][0], 3, 3, 3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            tmp3[i][j] += neg_skew_w_dt[i][j];
    block_set(&Phi[0][0], 6, 6, ESDIM, &tmp3[0][0], 3, 3, 3);

    /* Φ[6:9, 9:12] = -I·dt (∂δθ/∂δb_g) */
    block_set(&Phi[0][0], 6, 9, ESDIM, &neg_I_dt[0][0], 3, 3, 3);

    /* 计算 Φ·P·Φ^T */
    float PhiP[ESDIM][ESDIM], PhiPt[ESDIM][ESDIM], Pnew[ESDIM][ESDIM];
    m15_mul(Phi, ekf->P.data, PhiP);
    m15_trans(Phi, PhiPt);
    m15_mul(PhiP, PhiPt, Pnew);

    /* 加上 Q_d = diag(0, σ²_a·dt·I, σ²_g·dt·I, σ²_bg·dt·I, σ²_ba·dt·I) */
    float* n = &ekf->noise.gyro_noise; /* 取地址偏移 */
    /* 手动取各噪声参数 */
    float sg2 = ekf->noise.gyro_noise * ekf->noise.gyro_noise;              /* σ²_g  */
    float sa2 = ekf->noise.accel_noise * ekf->noise.accel_noise;            /* σ²_a  */
    float sbg2 = ekf->noise.gyro_bias_noise * ekf->noise.gyro_bias_noise;   /* σ²_bg */
    float sba2 = ekf->noise.accel_bias_noise * ekf->noise.accel_bias_noise; /* σ²_ba */

    /* Q_d 块:
     *   [3:6,  3:6]  += σ²_a · dt · I    (δv, 来自加速度计白噪声经 R^T 传播)
     *   [6:9,  6:9]  += σ²_g · dt · I    (δθ, 来自陀螺仪白噪声)
     *   [9:12, 9:12] += σ²_bg · dt · I   (δb_g, 来自 bias 随机游走)
     *   [12:15,12:15]+= σ²_ba · dt · I   (δb_a, 来自 bias 随机游走)
     */
    for (int i = 0; i < 3; i++) {
        Pnew[3 + i][3 + i] += sa2 * dt;
        Pnew[6 + i][6 + i] += sg2 * dt;
        Pnew[9 + i][9 + i] += sbg2 * dt;
        Pnew[12 + i][12 + i] += sba2 * dt;
    }

    /* 保证对称 */
    for (int i = 0; i < ESDIM; i++)
        for (int j = i + 1; j < ESDIM; j++) {
            float avg = 0.5f * (Pnew[i][j] + Pnew[j][i]);
            Pnew[i][j] = Pnew[j][i] = avg;
        }

    memcpy(ekf->P.data, Pnew, sizeof(ekf->P.data));
}

/* ========================================================================== */
/*  Section 6: 误差状态注入与重置                                              */
/* ========================================================================== */

/**
 * @brief 将误差状态注入标称状态, 然后重置误差状态为零
 *
 * 注入:
 *   pos_new  = pos  + δp
 *   vel_new  = vel  + δv
 *   q_new    = q    ⊗ [1, δθ/2]
 *   b_g_new  = b_g  + δb_g
 *   b_a_new  = b_a  + δb_a
 *
 * 重置协方差 (考虑四元数修正的雅可比):
 *   G_r = diag(I, I, I - 0.5·[δθ]×, I, I)
 *   P_new = G_r · P · G_r^T
 *
 * @param ekf   EKF 实例
 * @param dx    误差状态向量 (15 维)
 */
static void ekf_inject_and_reset(ekf_t* ekf, const float dx[ESDIM]) {
    /* ---- 注入标称状态 ---- */
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

    /* ---- 重置协方差 ---- */
    /*
     * G_r ≈ I - 0.5·[δθ]× (仅影响 δθ 对应的 [6:9, 6:9] 块)
     * 对于小 δθ, G_r ≈ I, 很多实现直接跳过此步。
     * 这里实现精确版本以保证数值正确性。
     */
    float dt_vec[3] = {dx[6], dx[7], dx[8]};
    float skew_dt[3][3];
    m3_skew(dt_vec, skew_dt);

    /* G_r 块: I - 0.5*[δθ]× */
    float Gr_theta[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Gr_theta[i][j] = (i == j ? 1.0f : 0.0f) - 0.5f * skew_dt[i][j];

    /* P_new = G_r · P · G_r^T
     * 仅 δθ 块受影响, 其余块不变。
     * 交叉项 (如 [δθ, δp]) 也需要变换:
     *   P[6:9, :] = Gr_theta · P[6:9, :]
     *   P[:, 6:9] = P[:, 6:9] · Gr_theta^T
     */
    float (*P)[ESDIM] = ekf->P.data;

    /* 变换行: P[6:9, j] = Gr_theta · P[6:9, j] */
    for (int j = 0; j < ESDIM; j++) {
        float col[3] = {P[6][j], P[7][j], P[8][j]};
        float out[3];
        m3_mul_v(Gr_theta, col, out);
        P[6][j] = out[0];
        P[7][j] = out[1];
        P[8][j] = out[2];
    }

    /* 变换列: P[i, 6:9] = P[i, 6:9] · Gr_theta^T */
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

    /* 保证对称 */
    for (int i = 0; i < ESDIM; i++)
        for (int j = i + 1; j < ESDIM; j++) {
            float avg = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = avg;
        }
}

/* ========================================================================== */
/*  Section 7: 通用 Kalman 更新                                                */
/* ========================================================================== */

/**
 * @brief 通用量测更新 (Joseph 形式)
 *
 * 算法:
 *   y  = z - h                   (新息)
 *   S  = H·P·H^T + R_noise       (新息协方差)
 *   K  = P·H^T·S^{-1}            (Kalman 增益)
 *   dx = K·y                     (误差状态修正)
 *   P  = (I-K·H)·P·(I-K·H)^T + K·R·K^T  (Joseph 形式)
 *
 * @param ekf       EKF 实例
 * @param z         量测向量 (dim_m 维)
 * @param h         预测量测 (dim_m 维)
 * @param H         量测雅可比 (dim_m × 15)
 * @param R_noise   量测噪声协方差 (dim_m × dim_m)
 * @param dim_m     量测维度 (1, 2, 或 3)
 */
static void ekf_update_generic(ekf_t* ekf,
                               const float z[],
                               const float h[],
                               const float H[][ESDIM],
                               const float R_noise[][MAX_MDIM],
                               int dim_m) {
    if (!ekf->initialized)
        return;

    float (*P)[ESDIM] = ekf->P.data;

    /* ---- Step 1: 新息 y = z - h ---- */
    float y[MAX_MDIM];
    for (int i = 0; i < dim_m; i++)
        y[i] = z[i] - h[i];

    /* ---- Step 2: PH^T = P · H^T (15 × dim_m) ---- */
    float PHt[ESDIM][MAX_MDIM];
    for (int i = 0; i < ESDIM; i++) {
        for (int j = 0; j < dim_m; j++) {
            float s = 0;
            for (int k = 0; k < ESDIM; k++)
                s += P[i][k] * H[j][k]; /* H[j][k] = H^T[k][j] */
            PHt[i][j] = s;
        }
    }

    /* ---- Step 3: S = H · PH^T + R (dim_m × dim_m) ---- */
    float S[MAX_MDIM][MAX_MDIM];
    for (int i = 0; i < dim_m; i++) {
        for (int j = 0; j < dim_m; j++) {
            float s = 0;
            for (int k = 0; k < ESDIM; k++)
                s += H[i][k] * PHt[k][j];
            S[i][j] = s + R_noise[i][j];
        }
    }

    /* ---- Step 4: S^{-1} ---- */
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
    } else { /* dim_m == 3 */
        if (m3_inv(S, S_inv) < EPS_F)
            return;
    }

    /* ---- Step 5: K = PH^T · S^{-1} (15 × dim_m) ---- */
    float K[ESDIM][MAX_MDIM];
    for (int i = 0; i < ESDIM; i++) {
        for (int j = 0; j < dim_m; j++) {
            float s = 0;
            for (int k = 0; k < dim_m; k++)
                s += PHt[i][k] * S_inv[k][j];
            K[i][j] = s;
        }
    }

    /* ---- Step 6: 新息卡方检验 (可选门限) ---- */
    /* chi2 = y^T · S^{-1} · y, 自由度 = dim_m */
    /* 这里做简单门限, 超过 5σ 则跳过更新 (避免发散) */
    float chi2 = 0;
    for (int i = 0; i < dim_m; i++) {
        float Sy_i = 0;
        for (int j = 0; j < dim_m; j++)
            Sy_i += S_inv[i][j] * y[j];
        chi2 += y[i] * Sy_i;
    }
    /* 3σ 门限: chi2 > dim_m * 9 对应 3σ (卡方分布) */
    float chi2_threshold = (float)dim_m * 9.0f;
    if (chi2 > chi2_threshold)
        return; /* 量测异常, 跳过 */

    /* ---- Step 7: dx = K · y ---- */
    float dx[ESDIM];
    for (int i = 0; i < ESDIM; i++) {
        float s = 0;
        for (int j = 0; j < dim_m; j++)
            s += K[i][j] * y[j];
        dx[i] = s;
    }

    /* ---- Step 8: 协方差更新 (Joseph 形式) ---- */
    /* A = I - K·H (15×15) */
    float A[ESDIM][ESDIM];
    m15_identity(A);
    for (int i = 0; i < ESDIM; i++)
        for (int j = 0; j < ESDIM; j++)
            for (int k = 0; k < dim_m; k++)
                A[i][j] -= K[i][k] * H[k][j];

    /* AP = A · P */
    float AP[ESDIM][ESDIM];
    m15_mul(A, ekf->P.data, AP);

    /* P_new = AP · A^T */
    float At[ESDIM][ESDIM], Pnew[ESDIM][ESDIM];
    m15_trans(A, At);
    m15_mul(AP, At, Pnew);

    /* K·R·K^T */
    /* KR[15][dim_m] = K · R */
    float KR[ESDIM][MAX_MDIM];
    for (int i = 0; i < ESDIM; i++) {
        for (int j = 0; j < dim_m; j++) {
            float s = 0;
            for (int k = 0; k < dim_m; k++)
                s += K[i][k] * R_noise[k][j];
            KR[i][j] = s;
        }
    }
    /* KRK = KR · K^T (15×15) */
    for (int i = 0; i < ESDIM; i++)
        for (int j = 0; j < ESDIM; j++)
            for (int k = 0; k < dim_m; k++)
                Pnew[i][j] += KR[i][k] * K[j][k];

    /* 对称化 */
    for (int i = 0; i < ESDIM; i++)
        for (int j = i + 1; j < ESDIM; j++) {
            float avg = 0.5f * (Pnew[i][j] + Pnew[j][i]);
            Pnew[i][j] = Pnew[j][i] = avg;
        }

    memcpy(ekf->P.data, Pnew, sizeof(ekf->P.data));

    /* ---- Step 9: 注入误差状态并重置 ---- */
    ekf_inject_and_reset(ekf, dx);
}

/* ========================================================================== */
/*  Section 8: 各传感器量测更新                                                */
/* ========================================================================== */

/* ------ 磁力计更新 (修正 yaw) ------ */

void ekf_update_mag(ekf_t* ekf, const ekf_mag_t* mag) {
    if (!ekf->initialized || !ekf->mag_ref.calibrated)
        return;
    if (mag->header.status != EKF_SENSOR_VALID)
        return;

    /* 预测量测: h = R · m_earth */
    float R[3][3], RT[3][3];
    ekf_get_rotmat(ekf, R, RT);

    float m_earth[3] = {
        ekf->mag_ref.m_earth.x,
        ekf->mag_ref.m_earth.y,
        ekf->mag_ref.m_earth.z};

    float h[3];
    /* h = R · m_earth, 但 R 是 world→body, 而 m_earth 在 world 系
     * h = R · m_earth (将世界系磁场转到机体系, 与量测比较)
     */
    m3_mul_v(R, m_earth, h);

    /* 量测: z = [m_x, m_y, m_z] (机体系) */
    float z[3] = {mag->m_x, mag->m_y, mag->m_z};

    /* 雅可比 H (3×15):
     *   ∂h/∂δθ = -R · [m_earth]×
     *   其余块为零
     */
    float H_mag[MAX_MDIM][ESDIM];
    memset(H_mag, 0, sizeof(H_mag));

    float skew_m[3][3], R_skew_m[3][3], neg_R_skew_m[3][3];
    m3_skew(m_earth, skew_m);
    m3_mul(R, skew_m, R_skew_m);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            neg_R_skew_m[i][j] = -R_skew_m[i][j];

    /* H[0:3, 6:9] = -R · [m_earth]× */
    block_set(&H_mag[0][0], 0, 6, ESDIM, &neg_R_skew_m[0][0], 3, 3, 3);

    /* 噪声协方差 R_noise */
    float sigma2 = ekf->noise.mag_noise * ekf->noise.mag_noise;
    float R_noise[MAX_MDIM][MAX_MDIM] = {
        {sigma2, 0, 0},
        {0, sigma2, 0},
        {0, 0, sigma2}};

    ekf_update_generic(ekf, z, h, H_mag, R_noise, 3);
}

/* ------ GPS 更新 (位置 + 速度) ------ */

void ekf_update_gps(ekf_t* ekf, const ekf_gps_t* gps) {
    if (!ekf->initialized)
        return;
    if (gps->header.status != EKF_SENSOR_VALID)
        return;
    if (gps->fix_type < 3)
        return; /* 需要 3D 定位 */
    if (gps->num_sats < 6)
        return; /* 至少 6 颗星 */

    /* 初始化 GPS 原点 */
    if (!ekf->gps_origin.initialized) {
        ekf_gps_origin_init(&ekf->gps_origin,
                            gps->latitude, gps->longitude, gps->altitude_msl);
        /* 第一帧 GPS 直接设为当前位置 */
        ekf->state.pos.x = 0;
        ekf->state.pos.y = 0;
        ekf->state.pos.z = 0;
        return;
    }

    /* ---- GPS 位置更新 ---- */
    ekf_vec3_t gps_ned;
    ekf_gps_to_ned(&ekf->gps_origin,
                   gps->latitude, gps->longitude, gps->altitude_msl,
                   &gps_ned);

    float z_pos[3] = {gps_ned.x, gps_ned.y, gps_ned.z};
    float h_pos[3] = {ekf->state.pos.x, ekf->state.pos.y, ekf->state.pos.z};

    /* H_pos = [I(3×3), 0, 0, 0, 0] */
    float H_pos[MAX_MDIM][ESDIM];
    memset(H_pos, 0, sizeof(H_pos));
    H_pos[0][0] = H_pos[1][1] = H_pos[2][2] = 1.0f;

    /* 噪声: 使用 GPS 报告精度或默认值 */
    float pos_var = gps->horiz_acc > 0 ? gps->horiz_acc : ekf->noise.gps_pos_noise;
    pos_var *= pos_var;
    float vert_var = gps->vert_acc > 0 ? gps->vert_acc : ekf->noise.gps_pos_noise * 1.5f;
    vert_var *= vert_var;

    float R_gps_pos[MAX_MDIM][MAX_MDIM] = {
        {pos_var, 0, 0},
        {0, pos_var, 0},
        {0, 0, vert_var}};

    ekf_update_generic(ekf, z_pos, h_pos, H_pos, R_gps_pos, 3);

    /* ---- GPS 速度更新 ---- */
    float z_vel[3] = {gps->vel_north, gps->vel_east, gps->vel_down};
    float h_vel[3] = {ekf->state.vel.x, ekf->state.vel.y, ekf->state.vel.z};

    /* H_vel = [0, I(3×3), 0, 0, 0] */
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

/* ------ 气压计更新 (高度) ------ */

void ekf_update_baro(ekf_t* ekf, const ekf_baro_t* baro) {
    if (!ekf->initialized)
        return;
    if (baro->header.status != EKF_SENSOR_VALID)
        return;

    /* 量测: z = baro.altitude (向上为正)
     * 预测: h = -p_D (NED D 轴取反)
     */
    float z[1] = {baro->altitude};
    float h_pred[1] = {-ekf->state.pos.z};

    /* H_baro = [0, 0, -1, 0, 0, ..., 0] (1×15) */
    float H_baro[1][ESDIM];
    memset(H_baro, 0, sizeof(H_baro));
    H_baro[0][2] = -1.0f; /* ∂h/∂δp_D = -1 */

    float sigma2 = ekf->noise.baro_noise * ekf->noise.baro_noise;
    float R_baro[MAX_MDIM][MAX_MDIM] = {{0}};
    R_baro[0][0] = sigma2;

    ekf_update_generic(ekf, z, h_pred, H_baro, R_baro, 1);
}

/* ------ 光流更新 (机体系水平速度) ------ */

void ekf_update_optflow(ekf_t* ekf, const ekf_optflow_t* flow) {
    if (!ekf->initialized)
        return;
    if (flow->header.status != EKF_SENSOR_VALID)
        return;
    if (flow->quality < 50)
        return; /* 质量太低, 跳过 */
    if (flow->distance_m <= 0)
        return; /* 无有效高度信息 */

    /* 光流输出已经是线速度 (rad/s × height = m/s) */
    float z[2] = {flow->integrated_x, flow->integrated_y};

    /* 预测量测: h = (R · v_world)[0:2] (机体系 XY 速度) */
    float R[3][3];
    ekf_get_rotmat(ekf, R, NULL);

    float v_world[3] = {
        ekf->state.vel.x, ekf->state.vel.y, ekf->state.vel.z};
    float v_body[3];
    m3_mul_v(R, v_world, v_body);

    float h_pred[2] = {v_body[0], v_body[1]};

    /* 雅可比 H (2×15):
     *   ∂h/∂δv = R[0:2, :]
     *   ∂h/∂δθ = -(R · [v_world]×)[0:2, :]
     */
    float H_flow[MAX_MDIM][ESDIM];
    memset(H_flow, 0, sizeof(H_flow));

    /* ∂h/∂δv: H[0:2, 3:6] = R 的前两行 */
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++)
            H_flow[i][3 + j] = R[i][j];

    /* ∂h/∂δθ: H[0:2, 6:9] = -(R · [v_world]×) 的前两行 */
    float skew_vw[3][3], R_skew_vw[3][3];
    m3_skew(v_world, skew_vw);
    m3_mul(R, skew_vw, R_skew_vw);

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++)
            H_flow[i][6 + j] = -R_skew_vw[i][j];

    /* 噪声: 取决于高度和光流质量 */
    float h_agl = flow->distance_m;
    float sigma_base = ekf->noise.optflow_noise / (h_agl > 0.3f ? h_agl : 0.3f);
    /* 质量越低, 噪声越大 */
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
