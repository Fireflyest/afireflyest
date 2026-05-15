#include "calibrate_accel.h"
#include <math.h>
#include <string.h>


// 初始化校准句柄
void Calib_Accel_Init(Calib_Accel_Handle_t *handle)
{
    memset(handle, 0, sizeof(Calib_Accel_Handle_t));
    handle->state = CALIB_ACCEL_IDLE;
    handle->current_face = 0;
    
    for (int face = 0; face < 6; face++) {
        for (int i = 0; i < 3; i++) {
            handle->face_min[face][i] = 1e9f;
            handle->face_max[face][i] = -1e9f;
        }
    }
}

// 开始校准
void Calib_Accel_Start(Calib_Accel_Handle_t *handle)
{
    handle->state = CALIB_ACCEL_COLLECTING;
    handle->current_face = 0;
    handle->sample_count = 0;
    
    for (int face = 0; face < 6; face++) {
        handle->face_samples_count[face] = 0;
        handle->face_done[face] = 0;

        for (int i = 0; i < 3; i++) {
            handle->face_min[face][i] = 1e9f;
            handle->face_max[face][i] = -1e9f;
        }
    }
}

// 添加样本
void Calib_Accel_AddSample(Calib_Accel_Handle_t *handle, const float accel[3], uint8_t face)
{
    if (handle->state != CALIB_ACCEL_COLLECTING) return;
    if (face >= 6) return;

    // 更新当前面的 min/max
    for (int i = 0; i < 3; i++) {
        if (accel[i] < handle->face_min[face][i]) {
            handle->face_min[face][i] = accel[i];
        }
        if (accel[i] > handle->face_max[face][i]) {
            handle->face_max[face][i] = accel[i];
        }
    }

    // 存储原始样本（用于最小二乘）
    if (handle->face_samples_count[face] < CALIB_ACCEL_FACE_SAMPLES) {
        handle->samples[handle->sample_count][0] = accel[0];
        handle->samples[handle->sample_count][1] = accel[1];
        handle->samples[handle->sample_count][2] = accel[2];
        handle->sample_count++;
    } else {
        handle->face_done[face] = 1;
        if (face >= 5) {
            Calib_Accel_Compute(handle);
        }
    }

    handle->face_samples_count[face]++;
}

uint8_t Calib_Accel_IsFaceDone(Calib_Accel_Handle_t *handle, uint8_t face) {
    return handle->face_done[face];
}

// 最小二乘法计算校准参数（6任意面校准）
void Calib_Accel_Compute(Calib_Accel_Handle_t *handle)
{
    const int n = handle->sample_count;

    /*
     * 参数向量 p = [bx, by, bz, sx, sy, sz]
     * 雅可比矩阵 J[k][i] = ∂r_k / ∂p_i
     *
     *   ∂r/∂bx = -2·sx²·(x - bx)
     *   ∂r/∂by = -2·sy²·(y - by)
     *   ∂r/∂bz = -2·sz²·(z - bz)
     *   ∂r/∂sx =  2·sx·(x - bx)²
     *   ∂r/∂sy =  2·sy·(y - by)²
     *   ∂r/∂sz =  2·sz·(z - bz)²
     *
     * 法方程: (JᵀJ)·Δp = -Jᵀr
     */

    float bias[3]  = {0.0f, 0.0f, 0.0f};
    float scale[3] = {1.0f, 1.0f, 1.0f};

    /* ---------- 初始值估计 ---------- */

    // 用每个轴的 min/max 粗估 bias
    for (int i = 0; i < 3; i++) {
        float lo =  1e9f, hi = -1e9f;
        for (int f = 0; f < 6; f++) {
            if (handle->face_min[f][i] < lo) lo = handle->face_min[f][i];
            if (handle->face_max[f][i] > hi) hi = handle->face_max[f][i];
        }
        if (hi - lo > 0.5f) {          // 该轴有足够量程变化
            bias[i]  = (hi + lo) * 0.5f;
            scale[i] = 2.0f / (hi - lo); // 期望量程 = ±1g → 总量程 2g
        }
    }

    /* ---------- Gauss-Newton 迭代 ---------- */

    for (int iter = 0; iter < 30; iter++) {

        // JᵀJ (6×6 对称) 与 Jᵀr (6×1)
        float JtJ[6][6] = {{0}};
        float Jtr[6]    = {0};

        for (int k = 0; k < n; k++) {
            float dx = handle->samples[k][0] - bias[0];
            float dy = handle->samples[k][1] - bias[1];
            float dz = handle->samples[k][2] - bias[2];

            float cx = dx * scale[0];           // 校准后分量
            float cy = dy * scale[1];
            float cz = dz * scale[2];

            float r = cx * cx + cy * cy + cz * cz - 1.0f;  // 残差

            // 雅可比行向量
            float J[6];
            J[0] = -2.0f * scale[0] * scale[0] * dx;       // ∂r/∂bx
            J[1] = -2.0f * scale[1] * scale[1] * dy;       // ∂r/∂by
            J[2] = -2.0f * scale[2] * scale[2] * dz;       // ∂r/∂bz
            J[3] =  2.0f * scale[0] * dx * dx;              // ∂r/∂sx
            J[4] =  2.0f * scale[1] * dy * dy;              // ∂r/∂sy
            J[5] =  2.0f * scale[2] * dz * dz;              // ∂r/∂sz

            // 累加 JᵀJ 和 Jᵀr
            for (int a = 0; a < 6; a++) {
                Jtr[a] += J[a] * r;
                for (int b = a; b < 6; b++) {
                    JtJ[a][b] += J[a] * J[b];
                }
            }
        }

        // 补全对称部分
        for (int a = 0; a < 6; a++)
            for (int b = a + 1; b < 6; b++)
                JtJ[b][a] = JtJ[a][b];

        // 轻微阻尼 (Levenberg 思想), 防止奇异
        for (int a = 0; a < 6; a++)
            JtJ[a][a] += JtJ[a][a] * 1e-4f + 1e-6f;

        /* ---------- 解 6×6 线性方程组 (列主元高斯消元) ---------- */

        // 增广矩阵 [ JtJ | -Jtr ]
        float aug[6][7];
        for (int a = 0; a < 6; a++) {
            for (int b = 0; b < 6; b++)
                aug[a][b] = JtJ[a][b];
            aug[a][6] = -Jtr[a];
        }

        // 消元
        for (int col = 0; col < 6; col++) {
            // 选主元
            int    best = col;
            float  best_val = fabsf(aug[col][col]);
            for (int row = col + 1; row < 6; row++) {
                float v = fabsf(aug[row][col]);
                if (v > best_val) { best_val = v; best = row; }
            }
            if (best != col) {
                for (int j = col; j < 7; j++) {
                    float t = aug[col][j]; aug[col][j] = aug[best][j]; aug[best][j] = t;
                }
            }
            if (fabsf(aug[col][col]) < 1e-12f) break;   // 奇异, 跳出

            // 消去下方行
            for (int row = col + 1; row < 6; row++) {
                float f = aug[row][col] / aug[col][col];
                for (int j = col; j < 7; j++)
                    aug[row][j] -= f * aug[col][j];
            }
        }

        // 回代
        float dp[6];
        for (int row = 5; row >= 0; row--) {
            if (fabsf(aug[row][row]) < 1e-12f) { dp[row] = 0; continue; }
            dp[row] = aug[row][6];
            for (int j = row + 1; j < 6; j++)
                dp[row] -= aug[row][j] * dp[j];
            dp[row] /= aug[row][row];
        }

        /* ---------- 更新参数 ---------- */

        // 步长限制: scale 变化不超过 20%, bias 变化有绝对上限
        float step = 1.0f;
        for (int i = 0; i < 3; i++) {
            if (scale[i] > 1e-6f) {
                float sc = fabsf(dp[i + 3]) / scale[i];
                if (sc > 0.2f && sc > step) step = 0.2f / sc;
            }
            float bc = fabsf(dp[i]);
            if (bc > 0.5f) {
                float s2 = 0.5f / bc;
                if (s2 < step) step = s2;
            }
        }

        for (int i = 0; i < 3; i++) {
            bias[i]  += dp[i]     * step;
            scale[i] += dp[i + 3] * step;
            if (scale[i] < 0.1f) scale[i] = 0.1f;   // 物理下限
            if (scale[i] > 10.0f) scale[i] = 10.0f;  // 物理上限
        }

        /* ---------- 收敛判断 ---------- */

        float max_dp = 0;
        for (int i = 0; i < 6; i++)
            if (fabsf(dp[i]) > max_dp) max_dp = fabsf(dp[i]);

        if (max_dp < 1e-6f)
            break;
    }

    handle->calib.bias[0]   = bias[0];
    handle->calib.bias[1]   = bias[1];
    handle->calib.bias[2]   = bias[2];
    handle->calib.scale[0]  = scale[0];
    handle->calib.scale[1]  = scale[1];
    handle->calib.scale[2]  = scale[2];
    handle->calib.is_valid  = 1;
    handle->state = CALIB_ACCEL_DONE;
}

// 应用校准参数
void Calib_Accel_Apply(const Calib_Accel_t *calib, const float raw[3], float calibrated[3])
{
    if (!calib->is_valid) {
        calibrated[0] = raw[0];
        calibrated[1] = raw[1];
        calibrated[2] = raw[2];
        return;
    }

    for (int i = 0; i < 3; i++) {
        calibrated[i] = (raw[i] - calib->bias[i]) * calib->scale[i];
    }
}
