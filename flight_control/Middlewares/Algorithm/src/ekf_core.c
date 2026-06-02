/**
 * @file ekf_core.c
 * @brief EKF 四旋翼无人机 — 完整实现
 *
 * Error-State EKF (ESEKF)，15 维误差状态
 * 所有数学推参见 ekf_types.h / ekf_sensors.h 头注释
 */

#include "ekf_core.h"
#include <math.h>
#include <string.h>

/* ========================================================================== */
/*  内部工具函数                                                               */
/* ========================================================================== */

/** @brief 3×3 反对称矩阵 S = [v×] */
static void skew_sym(const float v[3], float S[3][3]) {
    S[0][0] = 0.0f;
    S[0][1] = -v[2];
    S[0][2] = v[1];
    S[1][0] = v[2];
    S[1][1] = 0.0f;
    S[1][2] = -v[0];
    S[2][0] = -v[1];
    S[2][1] = v[0];
    S[2][2] = 0.0f;
}

/** @brief C = A × B (3×3) */
static void m3_mul(const float A[3][3], const float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
}

/** @brief out = A × v (3×3 × 3×1) */
static void m3_vec(const float A[3][3], const float v[3], float out[3]) {
    out[0] = A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2];
    out[1] = A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2];
    out[2] = A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2];
}

/** @brief 3×3 矩阵求逆，返回 0 成功 / -1 奇异 */
static int m3_inv(const float A[3][3], float Ai[3][3]) {
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (fabsf(det) < 1e-15f)
        return -1;
    float id = 1.0f / det;
    Ai[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * id;
    Ai[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) * id;
    Ai[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * id;
    Ai[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) * id;
    Ai[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * id;
    Ai[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) * id;
    Ai[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * id;
    Ai[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) * id;
    Ai[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * id;
    return 0;
}

/** @brief 2×2 对称矩阵求逆 */
static int m2_inv_sym(const float S[2][2], float Si[2][2]) {
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (fabsf(det) < 1e-15f)
        return -1;
    float id = 1.0f / det;
    Si[0][0] = S[1][1] * id;
    Si[0][1] = -S[0][1] * id;
    Si[1][0] = -S[1][0] * id;
    Si[1][1] = S[0][0] * id;
    return 0;
}

/* ========================================================================== */
/*  协方差传播（稀疏 Φ·P·Φᵀ）                                                  */
/* ========================================================================== */

/**
 * 预计算 Φ 的非平凡块并完成 P ← Φ·P·Φᵀ + Q_d
 *
 * Φ = I₁₅ + F·Δt，非零块:
 *   Φ[0][1] = Δt·I          (位置←速度)
 *   Φ[1][2] = -Rᵀ[a×]·Δt   (速度←姿态，加速度耦合)
 *   Φ[1][4] = -Rᵀ·Δt        (速度←加速度计bias)
 *   Φ[2][2] = I - [ω×]·Δt   (姿态自转)
 *   Φ[2][3] = -I·Δt          (姿态←陀螺仪bias)
 *   对角其余 = I
 *
 * Q_d = diag(0, σ²_a·Δt·I, σ²_g·Δt·I, σ²_bg·Δt·I, σ²_ba·Δt·I)
 */
static void propagate_covariance(ekf_t* ekf,
                                 const ekf_mat3_t* R,
                                 float ax,
                                 float ay,
                                 float az,
                                 float wx,
                                 float wy,
                                 float wz,
                                 float dt) {
    const ekf_noise_params_t* n = &ekf->noise;
    float (*P)[EKF_ERROR_STATE_DIM] = ekf->P.data;

    /* ---------- 预计算 Φ 的特殊块 ---------- */

    /* [a×] */
    float Sa[3][3];
    skew_sym((float[3]){ax, ay, az}, Sa);

    /* Rᵀ */
    float Rt[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Rt[i][j] = R->m[j][i];

    /* Φ12 = -Rᵀ·[a×]·Δt */
    float tmp33[3][3], Phi12[3][3];
    m3_mul(Rt, Sa, tmp33);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Phi12[i][j] = -tmp33[i][j] * dt;

    /* Φ14 = -Rᵀ·Δt */
    float Phi14[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Phi14[i][j] = -Rt[i][j] * dt;

    /* Φ22 = I - [ω×]·Δt */
    float Phi22[3][3];
    Phi22[0][0] = 1.0f;
    Phi22[0][1] = wz * dt;
    Phi22[0][2] = -wy * dt;
    Phi22[1][0] = -wz * dt;
    Phi22[1][1] = 1.0f;
    Phi22[1][2] = wx * dt;
    Phi22[2][0] = wy * dt;
    Phi22[2][1] = -wx * dt;
    Phi22[2][2] = 1.0f;

    /* Φ23 = -I·Δt */

    /* ---------- M = Φ·P (利用稀疏性逐行算) ---------- */
    /* 分 5×5 块 (每块 3×3)，行索引 bi=0..4, 列索引 bj=0..4 */
    /* Row 0: M₀ⱼ = P₀ⱼ + Δt·P₁ⱼ                                   */
    /* Row 1: M₁ⱼ = P₁ⱼ + Φ12·P₂ⱼ + Φ14·P₄ⱼ                       */
    /* Row 2: M₂ⱼ = Φ22·P₂ⱼ - Δt·P₃ⱼ                               */
    /* Row 3: M₃ⱼ = P₃ⱼ                                              */
    /* Row 4: M₄ⱼ = P₄ⱼ                                              */

    /* 为节省栈空间，先算 M 再直接算 P_new = M·Φᵀ + Q_d */
    float M[15][15];

    for (int bj = 0; bj < 5; bj++) {
        int cb = bj * 3;

        /* Row 0 */
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                M[r][cb + c] = P[r][cb + c] + dt * P[3 + r][cb + c];

        /* Row 1 */
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                float s = P[3 + r][cb + c];
                for (int k = 0; k < 3; k++)
                    s += Phi12[r][k] * P[6 + k][cb + c];
                for (int k = 0; k < 3; k++)
                    s += Phi14[r][k] * P[12 + k][cb + c];
                M[3 + r][cb + c] = s;
            }

        /* Row 2 */
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                float s = -dt * P[9 + r][cb + c];
                for (int k = 0; k < 3; k++)
                    s += Phi22[r][k] * P[6 + k][cb + c];
                M[6 + r][cb + c] = s;
            }

        /* Row 3, 4 — 直接拷贝 */
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                M[9 + r][cb + c] = P[9 + r][cb + c];
                M[12 + r][cb + c] = P[12 + r][cb + c];
            }
    }

/* ---------- P_new = M·Φᵀ + Q_d (利用对称性只算上三角) ---------- */
/* Φᵀ 的非零块 (列视角):                                             */
/*   Col 0: Φᵀ[0][0]=I                                             */
/*   Col 1: Φᵀ[1][0]=Δt·I, Φᵀ[1][1]=I                             */
/*   Col 2: Φᵀ[2][1]=Φ12ᵀ, Φᵀ[2][2]=Φ22ᵀ, Φᵀ[2][3]=-Δt·I       */
/*   Col 3: Φᵀ[3][2]=-Δt·I, Φᵀ[3][3]=I                            */
/*   Col 4: Φᵀ[4][1]=Φ14ᵀ, Φᵀ[4][4]=I                             */

/* 直接计算每个输出块，避免显式构造 Φᵀ */
#define BLK(BI, BJ, R, C) ((BI) * 3 + (R))[(BJ) * 3 + (C)]

    for (int bi = 0; bi < 5; bi++) {
        for (int bj = bi; bj < 5; bj++) {
            int ri = bi * 3, rj = bj * 3;

            for (int r = (bi == bj ? 0 : 0); r < 3; r++) {
                int c_start = (bi == bj) ? r : 0;
                for (int c = c_start; c < 3; c++) {
                    float s = 0.0f;

                    /* P_new[bi][bj] = Σ_k M[bi*3+r][k] · Φᵀ[k][bj*3+c] */
                    /* 即 M 的第 (bi,r) 行点乘 Φᵀ 的第 (bj,c) 列 */

                    /* k in block 0: Φᵀ[0*3+k][bj*3+c] = Φ[bj*3+c][0*3+k]^T */
                    /*   Φ[0][0]=I → Φᵀ[0][0]=I                           */
                    /*   Φ[1][0]=0 → Φᵀ[0][1]=0                           */
                    /*   Φ[2][0]=0 → Φᵀ[0][2]=0                           */
                    /*   Φ[3][0]=0 → Φᵀ[0][3]=0                           */
                    /*   Φ[4][0]=0 → Φᵀ[0][4]=0                           */
                    if (bj == 0) {
                        s += M[ri + r][rj + c]; /* I block */
                    }

                    /* k in block 1: Φᵀ[1*3+k][bj*3+c] */
                    /*   Φ[0][1]=ΔtI → Φᵀ[1][0]=ΔtI  (bj==0) */
                    /*   Φ[1][1]=I   → Φᵀ[1][1]=I    (bj==1) */
                    /*   Φ[2][1]=Φ12 → Φᵀ[1][2]=Φ12ᵀ (bj==2) */
                    /*   Φ[3][1]=0                    (bj==3) */
                    /*   Φ[4][1]=Φ14 → Φᵀ[1][4]=Φ14ᵀ (bj==4) */
                    if (bj == 0) {
                        s += M[ri + r][3 + c] * dt;
                    } else if (bj == 1) {
                        s += M[ri + r][3 + c];
                    }

                    /* k in block 2: Φᵀ[2*3+k][bj*3+c] */
                    /*   Φ[0][2]=0                       (bj==0) */
                    /*   Φ[1][2]=Φ12 → Φᵀ[2][1]=Φ12ᵀ    (bj==1) */
                    /*   Φ[2][2]=Φ22 → Φᵀ[2][2]=Φ22ᵀ    (bj==2) */
                    /*   Φ[3][2]=-ΔtI → Φᵀ[2][3]=-ΔtI   (bj==3) */
                    /*   Φ[4][2]=0                       (bj==4) */
                    if (bj == 1) {
                        for (int k = 0; k < 3; k++)
                            s += M[ri + r][6 + k] * Phi12[c][k];
                    } else if (bj == 2) {
                        for (int k = 0; k < 3; k++)
                            s += M[ri + r][6 + k] * Phi22[c][k];
                    }

                    /* k in block 3: Φᵀ[3*3+k][bj*3+c] */
                    /*   Φ[2][3]=-ΔtI → Φᵀ[3][2]=-ΔtI  (bj==2) */
                    /*   Φ[3][3]=I → Φᵀ[3][3]=I         (bj==3) */
                    /*   Φ[4][3]=0                       */
                    if (bj == 2) {
                        s += -dt * M[ri + r][9 + c];
                    } else if (bj == 3) {
                        s += M[ri + r][9 + c];
                    }

                    /* k in block 4: Φᵀ[4*3+k][bj*3+c] */
                    /*   Φ[1][4]=Φ14 → Φᵀ[4][1]=Φ14ᵀ    (bj==1) */
                    /*   Φ[4][4]=I → Φᵀ[4][4]=I         (bj==4) */
                    if (bj == 1) {
                        for (int k = 0; k < 3; k++)
                            s += M[ri + r][12 + k] * Phi14[c][k];
                    } else if (bj == 4) {
                        s += M[ri + r][12 + c];
                    }

                    /* Q_d (对角块，对角线元素) */
                    if (bi == bj && r == c) {
                        switch (bi) {
                        case 1:
                            s += n->accel_noise * n->accel_noise * dt;
                            break;
                        case 2:
                            s += n->gyro_noise * n->gyro_noise * dt;
                            break;
                        case 3:
                            s += n->gyro_bias_noise * n->gyro_bias_noise * dt;
                            break;
                        case 4:
                            s += n->accel_bias_noise * n->accel_bias_noise * dt;
                            break;
                        default:
                            break;
                        }
                    }

                    P[ri + r][rj + c] = s;
                    if (bi != bj || r != c)
                        P[rj + c][ri + r] = s; /* 对称 */
                }
            }
        }
    }

#undef BLK
}

/* ========================================================================== */
/*  通用 Kalman 更新 (Joseph 形式 + 野值门控)                                  */
/* ========================================================================== */

/**
 * @param n  观测维度 (1, 2, 3)
 * @param y  新息向量 [n]
 * @param H  观测矩阵 [n × 15] 行主序
 * @param R  量测噪声 [n × n] 对称，行主序
 */
static void ekf_do_update(ekf_t* ekf, int n, const float* y, const float* H, const float* R) {
    if (!ekf->initialized)
        return;

    float (*P)[EKF_ERROR_STATE_DIM] = ekf->P.data;
    const int N = EKF_ERROR_STATE_DIM; /* 15 */

    /* ---- Step 1: HP = H · P  (n × 15) ---- */
    float HP[3][15];
    for (int i = 0; i < n; i++)
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++)
                s += H[i * N + k] * P[k][j];
            HP[i][j] = s;
        }

    /* ---- Step 2: S = H·P·Hᵀ + R  (n × n, 对称) ---- */
    float S[3][3];
    for (int i = 0; i < n; i++)
        for (int j = i; j < n; j++) {
            float s = R[i * n + j];
            for (int k = 0; k < N; k++)
                s += HP[i][k] * H[j * N + k];
            S[i][j] = s;
            if (j != i)
                S[j][i] = s;
        }

    /* ---- Step 3: K = P·Hᵀ·S⁻¹  (15 × n) ---- */
    float Si[3][3];
    int singular = 0;
    if (n == 1) {
        if (fabsf(S[0][0]) < 1e-15f)
            singular = 1;
        else
            Si[0][0] = 1.0f / S[0][0];
    } else if (n == 2) {
        float S2[2][2] = {{S[0][0], S[0][1]},
                          {S[1][0], S[1][1]}};
        float Si2[2][2];
        singular = m2_inv_sym(S2, Si2);
        if (!singular) {
            Si[0][0] = Si2[0][0];
            Si[0][1] = Si2[0][1];
            Si[1][0] = Si2[1][0];
            Si[1][1] = Si2[1][1];
        }
    } else {
        singular = m3_inv(S, Si);
    }
    if (singular)
        return;

    /* PHᵀ 的第 (j,i) 元素 = P[j]ᵀ·H[i]ᵀ = HP[i][j] (P 对称) */
    float K[15][3];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < n; j++) {
            float s = 0.0f;
            for (int k = 0; k < n; k++)
                s += HP[k][i] * Si[k][j]; /* PHᵀ[i][k] = HP[k][i] */
            K[i][j] = s;
        }

    /* ---- Step 4: 野值门控 (Mahalanobis 距离) ---- */
    /* χ² 阈值: 99% 置信 */
    static const float chi2_thresh[] = {0.0f, 6.635f, 9.210f, 11.345f};
    float d2 = 0.0f;
    for (int i = 0; i < n; i++) {
        float Syi = 0.0f;
        for (int j = 0; j < n; j++)
            Syi += S[i][j] * y[j];
        d2 += y[i] * Syi;
    }
    if (d2 > chi2_thresh[n])
        return;

    /* ---- Step 5: 误差状态 δx = K · y ---- */
    float dx[15];
    for (int i = 0; i < N; i++) {
        float s = 0.0f;
        for (int j = 0; j < n; j++)
            s += K[i][j] * y[j];
        dx[i] = s;
    }

    /* ---- Step 6: 修正标称状态 ---- */
    ekf->state.pos.x += dx[0];
    ekf->state.pos.y += dx[1];
    ekf->state.pos.z += dx[2];
    ekf->state.vel.x += dx[3];
    ekf->state.vel.y += dx[4];
    ekf->state.vel.z += dx[5];

    ekf_vec3_t dtheta = {dx[6], dx[7], dx[8]};
    ekf_quat_t q_corr;
    ekf_quat_apply_correction(&ekf->state.quat, &dtheta, &q_corr);
    ekf->state.quat = q_corr;

    ekf->state.gyro_bias.x += dx[9];
    ekf->state.gyro_bias.y += dx[10];
    ekf->state.gyro_bias.z += dx[11];
    ekf->state.accel_bias.x += dx[12];
    ekf->state.accel_bias.y += dx[13];
    ekf->state.accel_bias.z += dx[14];

    /* ---- Step 7: Joseph 形式协方差更新 ---- */
    /* P_new = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ                 */
    /*       = AP  - AP·Hᵀ·Kᵀ  + K·R·Kᵀ                  */
    /* 其中 AP = P - K·HP = (I-KH)·P                       */

    /* AP = P - K·HP  (15×15) */
    float AP[15][15];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < n; k++)
                s += K[i][k] * HP[k][j];
            AP[i][j] = P[i][j] - s;
        }

    /* AP_Ht = AP · Hᵀ  (15 × n) */
    float AP_Ht[15][3];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < n; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++)
                s += AP[i][k] * H[j * N + k]; /* Hᵀ[k][j] = H[j*N+k] */
            AP_Ht[i][j] = s;
        }

    /* KR = K · R  (15 × n) */
    float KR[15][3];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < n; j++) {
            float s = 0.0f;
            for (int k = 0; k < n; k++)
                s += K[i][k] * R[k * n + j];
            KR[i][j] = s;
        }

    /* P_new[i][j] = AP[i][j] + Σ_k (KR[i][k] - AP_Ht[i][k]) · K[j][k] */
    for (int i = 0; i < N; i++)
        for (int j = i; j < N; j++) {
            float s = AP[i][j];
            for (int k = 0; k < n; k++)
                s += (KR[i][k] - AP_Ht[i][k]) * K[j][k];
            P[i][j] = s;
            if (j != i)
                P[j][i] = s;
        }
}

/* ========================================================================== */
/*  ekf_types.h — 四元数与状态工具实现                                          */
/* ========================================================================== */

void ekf_quat_normalize(ekf_quat_t* q) {
    float n2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    if (n2 < 1e-20f) {
        q->w = 1.0f;
        q->x = q->y = q->z = 0.0f;
        return;
    }
    float inv = 1.0f / sqrtf(n2);
    q->w *= inv;
    q->x *= inv;
    q->y *= inv;
    q->z *= inv;
}

void ekf_quat_to_rotmat(const ekf_quat_t* q, ekf_mat3_t* R) {
    float w = q->w, x = q->x, y = q->y, z = q->z;
    float x2 = x * x, y2 = y * y, z2 = z * z;
    float wx = w * x, wy = w * y, wz = w * z;
    float xy = x * y, xz = x * z, yz = y * z;

    R->m[0][0] = 1.0f - 2.0f * (y2 + z2);
    R->m[0][1] = 2.0f * (xy + wz);
    R->m[0][2] = 2.0f * (xz - wy);
    R->m[1][0] = 2.0f * (xy - wz);
    R->m[1][1] = 1.0f - 2.0f * (x2 + z2);
    R->m[1][2] = 2.0f * (yz + wx);
    R->m[2][0] = 2.0f * (xz + wy);
    R->m[2][1] = 2.0f * (yz - wx);
    R->m[2][2] = 1.0f - 2.0f * (x2 + y2);
}

void ekf_quat_to_euler(const ekf_quat_t* q, ekf_euler_t* euler) {
    float w = q->w, x = q->x, y = q->y, z = q->z;

    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    euler->roll = atan2f(sinr, cosr);

    float sinp = 2.0f * (w * y - z * x);
    euler->pitch = (fabsf(sinp) >= 1.0f)
                       ? copysignf(M_PI_F * 0.5f, sinp)
                       : asinf(sinp);

    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);
    euler->yaw = atan2f(siny, cosy);
}

void ekf_euler_to_quat(const ekf_euler_t* euler, ekf_quat_t* q) {
    float hr = euler->roll * 0.5f;
    float hp = euler->pitch * 0.5f;
    float hy = euler->yaw * 0.5f;
    float cr = cosf(hr), sr = sinf(hr), cp = cosf(hp), sp = sinf(hp);
    float cy = cosf(hy), sy = sinf(hy);

    q->w = cr * cp * cy - sr * sp * sy;
    q->x = sr * cp * cy + cr * sp * sy;
    q->y = cr * sp * cy - sr * cp * sy;
    q->z = cr * cp * sy + sr * sp * cy;
}

void ekf_quat_mult(const ekf_quat_t* q1, const ekf_quat_t* q2, ekf_quat_t* o) {
    o->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    o->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    o->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    o->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void ekf_quat_apply_correction(const ekf_quat_t* q,
                               const ekf_vec3_t* delta,
                               ekf_quat_t* o) {
    /* q_out = q ⊗ [1, δθ/2]  一阶近似 */
    float hx = 0.5f * delta->x;
    float hy = 0.5f * delta->y;
    float hz = 0.5f * delta->z;

    o->w = q->w - q->x * hx - q->y * hy - q->z * hz;
    o->x = q->x + q->w * hx + q->y * hz - q->z * hy;
    o->y = q->y - q->x * hz + q->w * hy + q->z * hx;
    o->z = q->z + q->x * hy - q->y * hx + q->w * hz;
    ekf_quat_normalize(o);
}

void ekf_state_init_default(ekf_state_t* s) {
    s->pos.x = s->pos.y = s->pos.z = 0.0f;
    s->vel.x = s->vel.y = s->vel.z = 0.0f;
    s->quat.w = 1.0f;
    s->quat.x = s->quat.y = s->quat.z = 0.0f;
    s->gyro_bias.x = s->gyro_bias.y = s->gyro_bias.z = 0.0f;
    s->accel_bias.x = s->accel_bias.y = s->accel_bias.z = 0.0f;
}

void ekf_cov_init_diagonal(ekf_cov_t* P,
                           float pos_std,
                           float vel_std,
                           float att_std,
                           float gbias_std,
                           float abias_std) {
    memset(P->data, 0, sizeof(P->data));
    for (int i = 0; i < 3; i++) {
        P->data[0 + i][0 + i] = pos_std * pos_std;       /* δp */
        P->data[3 + i][3 + i] = vel_std * vel_std;       /* δv */
        P->data[6 + i][6 + i] = att_std * att_std;       /* δθ */
        P->data[9 + i][9 + i] = gbias_std * gbias_std;   /* δbg */
        P->data[12 + i][12 + i] = abias_std * abias_std; /* δba */
    }
}

/* ========================================================================== */
/*  ekf_sensors.h — 传感器工具实现                                             */
/* ========================================================================== */

void ekf_noise_params_init_default(ekf_noise_params_t* p) {
    p->gyro_noise = 1.0e-2f;       /* rad/s/√Hz   */
    p->gyro_bias_noise = 1.0e-4f;  /* rad/s/√s    */
    p->accel_noise = 5.0e-2f;      /* m/s²/√Hz    */
    p->accel_bias_noise = 1.0e-3f; /* m/s²/√s     */
    p->mag_noise = 0.5f;           /* μT/√Hz      */
    p->baro_noise = 0.5f;          /* m/√Hz       */
    p->baro_bias_noise = 5.0e-2f;  /* m/√s        */
    p->gps_pos_noise = 2.0f;       /* m (1σ)      */
    p->gps_vel_noise = 0.5f;       /* m/s (1σ)    */
    p->optflow_noise = 0.05f;      /* (m/s)/√Hz   */
}

void ekf_gps_to_ned(const ekf_gps_origin_t* origin,
                    double lat,
                    double lon,
                    float alt,
                    ekf_vec3_t* ned) {
    /* 小角度近似，距原点 10 km 内精度 < 1 m */
    const double R = 6371000.0;
    double dlat = (lat - origin->lat0) * (double)M_PI_F / 180.0;
    double dlon = (lon - origin->lon0) * (double)M_PI_F / 180.0;
    double cos_lat0 = cos(origin->lat0 * (double)M_PI_F / 180.0);

    ned->x = (float)(dlat * R);            /* N */
    ned->y = (float)(dlon * R * cos_lat0); /* E */
    ned->z = -(alt - origin->alt0);        /* D = -alt */
}

void ekf_gps_origin_init(ekf_gps_origin_t* origin,
                         double lat,
                         double lon,
                         float alt) {
    if (origin->initialized)
        return;
    origin->lat0 = lat;
    origin->lon0 = lon;
    origin->alt0 = alt;
    origin->initialized = 1;
}

/* ========================================================================== */
/*  ekf_core.h — 初始化 / 对准                                                 */
/* ========================================================================== */

void ekf_init(ekf_t* ekf, const ekf_noise_params_t* noise) {
    memset(ekf, 0, sizeof(ekf_t));

    if (noise)
        ekf->noise = *noise;
    else
        ekf_noise_params_init_default(&ekf->noise);

    ekf_state_init_default(&ekf->state);
    /* 合理的初始不确定度 */
    ekf_cov_init_diagonal(&ekf->P,
                          10.0f,  /* 位置 10 m    */
                          1.0f,   /* 速度 1 m/s   */
                          0.17f,  /* 姿态 ~10°    */
                          0.01f,  /* gyro bias    */
                          0.05f); /* accel bias   */
}

void ekf_set_init_altitude(ekf_t* ekf, float altitude) {
    ekf->state.pos.z = -altitude; /* NED: D = -altitude */
}

int ekf_align(ekf_t* ekf,
              const ekf_imu_t imu[],
              int imu_n,
              const ekf_mag_t mag[],
              int mag_n) {
    if (imu_n < 10)
        return -1;

    /* ---- 均值 ---- */
    float ax = 0, ay = 0, az = 0, wx = 0, wy = 0, wz = 0;
    for (int i = 0; i < imu_n; i++) {
        ax += imu[i].accel.a_x;
        ay += imu[i].accel.a_y;
        az += imu[i].accel.a_z;
        wx += imu[i].gyro.omega_x;
        wy += imu[i].gyro.omega_y;
        wz += imu[i].gyro.omega_z;
    }
    float inv_n = 1.0f / (float)imu_n;
    ax *= inv_n;
    ay *= inv_n;
    az *= inv_n;
    wx *= inv_n;
    wy *= inv_n;
    wz *= inv_n;

    /* 加速度幅值检查 */
    float a_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (a_norm < EKF_GRAVITY * 0.5f)
        return -2; /* 疑似自由落体/震动 */

    /* ---- Roll / Pitch ---- */
    float roll = atan2f(-ay, -az);
    float pitch = atan2f(ax, sqrtf(ay * ay + az * az));

    /* ---- Yaw (磁力计) ---- */
    float yaw = 0.0f;
    if (mag && mag_n > 0 && ekf->mag_ref.calibrated) {
        float mx = 0, my = 0, mz = 0;
        for (int i = 0; i < mag_n; i++) {
            mx += mag[i].m_x;
            my += mag[i].m_y;
            mz += mag[i].m_z;
        }
        float inv_m = 1.0f / (float)mag_n;
        mx *= inv_m;
        my *= inv_m;
        mz *= inv_m;

        /* 去倾斜: R_partial^T · m_body  (body→level) */
        float cr = cosf(roll), sr = sinf(roll);
        float cp = cosf(pitch), sp = sinf(pitch);

        /* t = Rx(-roll) · m */
        float t0 = mx;
        float t1 = cr * my - sr * mz;
        float t2 = sr * my + cr * mz;

        /* m_level = Ry(-pitch) · t */
        float ml0 = cp * t0 + sp * t2;
        float ml1 = t1;

        /* yaw = atan2(ml·mE - ml·mN, ml·mN + ml·mE) */
        float mN = ekf->mag_ref.m_earth.x;
        float mE = ekf->mag_ref.m_earth.y;
        yaw = atan2f(ml0 * mE - ml1 * mN, ml0 * mN + ml1 * mE);
    }

    /* ---- 设四元数 ---- */
    ekf_euler_t euler = {roll, pitch, yaw};
    ekf_euler_to_quat(&euler, &ekf->state.quat);

    /* ---- 设 bias ---- */
    ekf->state.gyro_bias.x = wx;
    ekf->state.gyro_bias.y = wy;
    ekf->state.gyro_bias.z = wz;
    /* accel bias 初始为 0，靠在线估计 */

    /* 重新初始化协方差 (对准后不确定度降低) */
    ekf_cov_init_diagonal(&ekf->P,
                          5.0f, 0.5f, 0.05f, 0.005f, 0.05f);

    ekf->initialized = 1;
    return 0;
}

/* ========================================================================== */
/*  ekf_core.h — 预测                                                         */
/* ========================================================================== */

void ekf_predict(ekf_t* ekf, const ekf_imu_t* imu) {
    /* 计算 Δt */
    float dt = 0.0f;
    if (ekf->last_predict_us > 0) {
        dt = (float)(imu->header.timestamp_us - ekf->last_predict_us) * 1e-6f;
        if (dt <= 0.0f || dt > 0.1f) {
            ekf->last_predict_us = imu->header.timestamp_us;
            return; /* 跳过异常帧 */
        }
    }
    ekf->last_predict_us = imu->header.timestamp_us;
    if (dt == 0.0f || !ekf->initialized)
        return;

    /* ---- 1. 去偏测量 ---- */
    float wx = imu->gyro.omega_x - ekf->state.gyro_bias.x;
    float wy = imu->gyro.omega_y - ekf->state.gyro_bias.y;
    float wz = imu->gyro.omega_z - ekf->state.gyro_bias.z;
    float ax = imu->accel.a_x - ekf->state.accel_bias.x;
    float ay = imu->accel.a_y - ekf->state.accel_bias.y;
    float az = imu->accel.a_z - ekf->state.accel_bias.z;

    /* ---- 2. 四元数精确积分 (指数映射) ---- */
    float omega2 = wx * wx + wy * wy + wz * wz;
    ekf_quat_t dq;
    if (omega2 > 1e-12f) {
        float omega = sqrtf(omega2);
        float half_ang = omega * dt * 0.5f;
        float s = sinf(half_ang) / omega;
        dq.w = cosf(half_ang);
        dq.x = s * wx;
        dq.y = s * wy;
        dq.z = s * wz;
    } else {
        dq.w = 1.0f;
        dq.x = wx * dt * 0.5f;
        dq.y = wy * dt * 0.5f;
        dq.z = wz * dt * 0.5f;
    }

    ekf_quat_t q_new;
    ekf_quat_mult(&ekf->state.quat, &dq, &q_new);
    ekf->state.quat = q_new;
    ekf_quat_normalize(&ekf->state.quat);

    /* ---- 3. 计算旋转矩阵 & 地球系加速度 ---- */
    ekf_mat3_t R;
    ekf_quat_to_rotmat(&ekf->state.quat, &R);

    /* a_earth = Rᵀ · a_body + [0, 0, +g]   (NED 重力向下) */
    float a_earth_x = R.m[0][0] * ax + R.m[1][0] * ay + R.m[2][0] * az;
    float a_earth_y = R.m[0][1] * ax + R.m[1][1] * ay + R.m[2][1] * az;
    float a_earth_z = R.m[0][2] * ax + R.m[1][2] * ay + R.m[2][2] * az + EKF_GRAVITY;

    /* ---- 4. 位置 / 速度中点法积分 ---- */
    ekf->state.pos.x += (ekf->state.vel.x + 0.5f * a_earth_x * dt) * dt;
    ekf->state.pos.y += (ekf->state.vel.y + 0.5f * a_earth_y * dt) * dt;
    ekf->state.pos.z += (ekf->state.vel.z + 0.5f * a_earth_z * dt) * dt;

    ekf->state.vel.x += a_earth_x * dt;
    ekf->state.vel.y += a_earth_y * dt;
    ekf->state.vel.z += a_earth_z * dt;

    /* ---- 5. 协方差传播 ---- */
    propagate_covariance(ekf, &R, ax, ay, az, wx, wy, wz, dt);
}

/* ========================================================================== */
/*  ekf_core.h — 量测更新                                                     */
/* ========================================================================== */

void ekf_update_gravity(ekf_t* ekf, const ekf_imu_t* imu) {
    if (!ekf->initialized)
        return;

    /* 重力方向观测: h(x) = R · g_spec,  g_spec = [0, 0, -g] */
    ekf_mat3_t R;
    ekf_quat_to_rotmat(&ekf->state.quat, &R);

    float g_spec[3] = {0.0f, 0.0f, -EKF_GRAVITY};
    float hg[3];
    m3_vec(R.m, g_spec, hg); /* 预测的加速度计输出 */

    float ax = imu->accel.a_x - ekf->state.accel_bias.x;
    float ay = imu->accel.a_y - ekf->state.accel_bias.y;
    float az = imu->accel.a_z - ekf->state.accel_bias.z;

    /* 自适应噪声: 高机动时降低对重力的信任 */
    float a_norm = sqrtf(ax * ax + ay * ay + az * az);
    float ratio = fabsf(a_norm / EKF_GRAVITY - 1.0f);
    float scale = 1.0f + ratio * ratio * 200.0f; /* 调参 */

    float y[3] = {ax - hg[0], ay - hg[1], az - hg[2]};

    /* H = [0 0 [R·g_spec ×]  0  0]  (3×15) */
    float skew_hg[3][3];
    skew_sym(hg, skew_hg);

    float H[3][15];
    memset(H, 0, sizeof(H));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            H[i][6 + j] = skew_hg[i][j];

    float Rm = ekf->noise.accel_noise * ekf->noise.accel_noise * scale;
    float R_mat[3][3] = {{Rm, 0, 0}, {0, Rm, 0}, {0, 0, Rm}};

    ekf_do_update(ekf, 3, y, &H[0][0], &R_mat[0][0]);
}

void ekf_update_mag(ekf_t* ekf, const ekf_mag_t* mag) {
    if (!ekf->initialized || !ekf->mag_ref.calibrated)
        return;

    ekf_mat3_t R;
    ekf_quat_to_rotmat(&ekf->state.quat, &R);

    /* h(x) = R · m_earth */
    float m_earth[3] = {ekf->mag_ref.m_earth.x,
                        ekf->mag_ref.m_earth.y,
                        ekf->mag_ref.m_earth.z};
    float hm[3];
    m3_vec(R.m, m_earth, hm);

    float y[3] = {mag->m_x - hm[0], mag->m_y - hm[1], mag->m_z - hm[2]};

    /* H = [0 0 [R·m_earth ×] 0 0] */
    float skew_hm[3][3];
    skew_sym(hm, skew_hm);

    float H[3][15];
    memset(H, 0, sizeof(H));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            H[i][6 + j] = skew_hm[i][j];

    float Rm = ekf->noise.mag_noise * ekf->noise.mag_noise;
    float R_mat[3][3] = {{Rm, 0, 0}, {0, Rm, 0}, {0, 0, Rm}};

    ekf_do_update(ekf, 3, y, &H[0][0], &R_mat[0][0]);
}

void ekf_update_gps(ekf_t* ekf, const ekf_gps_t* gps) {
    if (!ekf->initialized)
        return;
    if (gps->fix_type < 3 || gps->num_sats < 6)
        return;

    /* 首次有效 GPS → 初始化原点 */
    if (!ekf->gps_origin.initialized) {
        ekf_gps_origin_init(&ekf->gps_origin,
                            gps->latitude, gps->longitude, gps->altitude_msl);
    }

    /* WGS-84 → NED */
    ekf_vec3_t p_ned;
    ekf_gps_to_ned(&ekf->gps_origin,
                   gps->latitude, gps->longitude, gps->altitude_msl,
                   &p_ned);

    /* ---- 位置更新 (3 维) ---- */
    float y_pos[3] = {
        p_ned.x - ekf->state.pos.x,
        p_ned.y - ekf->state.pos.y,
        p_ned.z - ekf->state.pos.z};

    float H_pos[3][15];
    memset(H_pos, 0, sizeof(H_pos));
    H_pos[0][0] = H_pos[1][1] = H_pos[2][2] = 1.0f;

    float rp = gps->horiz_acc > 0.01f ? gps->horiz_acc : ekf->noise.gps_pos_noise;
    float rp_z = gps->vert_acc > 0.01f ? gps->vert_acc : ekf->noise.gps_pos_noise;
    float R_pos[3][3] = {{rp * rp, 0, 0},
                         {0, rp * rp, 0},
                         {0, 0, rp_z * rp_z}};

    ekf_do_update(ekf, 3, y_pos, &H_pos[0][0], &R_pos[0][0]);

    /* ---- 速度更新 (3 维) ---- */
    float y_vel[3] = {
        gps->vel_north - ekf->state.vel.x,
        gps->vel_east - ekf->state.vel.y,
        gps->vel_down - ekf->state.vel.z};

    float H_vel[3][15];
    memset(H_vel, 0, sizeof(H_vel));
    H_vel[0][3] = H_vel[1][4] = H_vel[2][5] = 1.0f;

    float rv = gps->vel_acc > 0.01f ? gps->vel_acc : ekf->noise.gps_vel_noise;
    float R_vel[3][3] = {{rv * rv, 0, 0},
                         {0, rv * rv, 0},
                         {0, 0, rv * rv}};

    ekf_do_update(ekf, 3, y_vel, &H_vel[0][0], &R_vel[0][0]);
}

void ekf_update_baro(ekf_t* ekf, const ekf_baro_t* baro) {
    if (!ekf->initialized)
        return;

    /* 首次接收气压计 → 记录初始偏移 */
    if (!ekf->baro_altitude_initialized) {
        ekf->baro_alt_offset = baro->altitude + ekf->state.pos.z;
        ekf->baro_altitude_initialized = 1;
        return;
    }

    /* h(x) = -p_D,  z = altitude_relative */
    float alt_rel = baro->altitude - ekf->baro_alt_offset;
    float y = alt_rel - (-ekf->state.pos.z); /* = alt_rel + p_D */

    /* H = [0 0 -1 0 ... 0]  (1×15) */
    float H[15];
    memset(H, 0, sizeof(H));
    H[2] = -1.0f;

    float R = ekf->noise.baro_noise * ekf->noise.baro_noise;

    ekf_do_update(ekf, 1, &y, H, &R);
}

void ekf_update_optflow(ekf_t* ekf, const ekf_optflow_t* flow) {
    if (!ekf->initialized)
        return;
    if (flow->quality < 50)
        return; /* 质量太低，跳过 */

    ekf_mat3_t R;
    ekf_quat_to_rotmat(&ekf->state.quat, &R);

    /* 预测: v_body_xy = (R · v_world)[0:2] */
    float vw[3] = {ekf->state.vel.x, ekf->state.vel.y, ekf->state.vel.z};
    float vb[3];
    m3_vec(R.m, vw, vb);

    float y[2] = {flow->velocity_x - vb[0], flow->velocity_y - vb[1]};

    /* H = [0₂ₓ₃  R[0:2,:]  [R·v×][0:2,:]  0₂ₓ₃  0₂ₓ₃] */
    float skew_vb[3][3];
    skew_sym(vb, skew_vb);

    float H[2][15];
    memset(H, 0, sizeof(H));
    for (int c = 0; c < 3; c++) {
        H[0][3 + c] = R.m[0][c];
        H[1][3 + c] = R.m[1][c];
        H[0][6 + c] = skew_vb[0][c];
        H[1][6 + c] = skew_vb[1][c];
    }

    float rn = ekf->noise.optflow_noise * ekf->noise.optflow_noise;
    /* 高度越大光流越不可靠 */
    if (flow->distance_m > 0.0f)
        rn *= (1.0f + flow->distance_m * flow->distance_m * 0.1f);

    float R_mat[2][2] = {{rn, 0}, {0, rn}};

    ekf_do_update(ekf, 2, y, &H[0][0], &R_mat[0][0]);
}

/* ========================================================================== */
/*  ekf_core.h — 状态读取                                                     */
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
    return ekf->initialized ? 1 : 0;
}
