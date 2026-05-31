#include "calibrate_mag.h"
#include <math.h>
#include <string.h>

/* ========================================================================== */
/*  内部: 计算校准参数                                                          */
/* ========================================================================== */

static void Calib_Mag_Compute(Calib_Mag_Handle_t* handle) {
    float radius[3];

    /* 硬铁偏置: 球心 = (max + min) / 2 */
    for (int i = 0; i < 3; i++) {
        handle->calib.bias[i] =
            (handle->mag_max[i] + handle->mag_min[i]) * 0.5f;
        radius[i] =
            (handle->mag_max[i] - handle->mag_min[i]) * 0.5f;
    }

    /* 安全检查: 各轴半径至少 5 μT (避免除零和病态拟合) */
    for (int i = 0; i < 3; i++) {
        if (radius[i] < 5.0f) {
            handle->state = CALIB_MAG_FAILED;
            return;
        }
    }

    /* 软铁比例因子: scale = avg_radius / radius[i]
     * 理想球体各轴半径相等, scale 全为 1
     * 实际 PCB 走线/结构件导致各轴不等, scale 补偿差异 */
    float avg_radius = (radius[0] + radius[1] + radius[2]) / 3.0f;
    for (int i = 0; i < 3; i++) {
        handle->calib.scale[i] = avg_radius / radius[i];
    }

    /* 记录总场强度 (用于健康检查) */
    handle->calib.total_field = avg_radius;

    /* 安全检查: 总场强度合理性 (地球磁场约 25~65 μT) */
    if (avg_radius < 20.0f || avg_radius > 100.0f) {
        handle->state = CALIB_MAG_FAILED;
        return;
    }

    handle->calib.is_valid = 1;
    handle->state = CALIB_MAG_DONE;
}

/* ========================================================================== */
/*  公共 API                                                                   */
/* ========================================================================== */

void Calib_Mag_Init(Calib_Mag_Handle_t* handle) {
    memset(handle, 0, sizeof(Calib_Mag_Handle_t));
    handle->state = CALIB_MAG_IDLE;
}

void Calib_Mag_Start(Calib_Mag_Handle_t* handle) {
    handle->state = CALIB_MAG_COLLECTING;
    handle->sample_count = 0;
    handle->coverage_ok = 0;
    handle->calib.is_valid = 0;

    /* 初始化 min 为最大值, max 为最小值, 使首次采样必定更新 */
    for (int i = 0; i < 3; i++) {
        handle->mag_min[i] = 1e6f;
        handle->mag_max[i] = -1e6f;
    }
}

void Calib_Mag_AddSample(Calib_Mag_Handle_t* handle, const float mag[3]) {
    if (handle->state != CALIB_MAG_COLLECTING)
        return;

    /* 更新 min/max */
    for (int i = 0; i < 3; i++) {
        if (mag[i] < handle->mag_min[i])
            handle->mag_min[i] = mag[i];
        if (mag[i] > handle->mag_max[i])
            handle->mag_max[i] = mag[i];
    }
    handle->sample_count++;

    /* 检查各轴覆盖度 */
    uint8_t ok = 1;
    for (int i = 0; i < 3; i++) {
        float range = handle->mag_max[i] - handle->mag_min[i];
        if (range < CALIB_MAG_MIN_RANGE) {
            ok = 0;
            break;
        }
    }
    handle->coverage_ok = ok;

    /* 样本足够 + 覆盖度达标 → 计算 */
    if (handle->sample_count >= CALIB_MAG_MIN_SAMPLES &&
        handle->coverage_ok) {
        Calib_Mag_Compute(handle);
        return;
    }

    /* 样本用尽仍未达标 → 失败 */
    if (handle->sample_count >= CALIB_MAG_MAX_SAMPLES) {
        handle->state = CALIB_MAG_FAILED;
    }
}

float Calib_Mag_GetProgress(Calib_Mag_Handle_t* handle) {
    if (handle->state != CALIB_MAG_COLLECTING)
        return 0.0f;

    /* 进度 = 各轴覆盖度的最小值 (range / MIN_RANGE), 上限 1.0 */
    float min_ratio = 1e6f;
    for (int i = 0; i < 3; i++) {
        float range = handle->mag_max[i] - handle->mag_min[i];
        float ratio = range / CALIB_MAG_MIN_RANGE;
        if (ratio < min_ratio)
            min_ratio = ratio;
    }

    /* 同时考虑样本进度 */
    float sample_ratio = (float)handle->sample_count / CALIB_MAG_MIN_SAMPLES;

    /* 取两者中较小的 (都必须达标) */
    float progress = (min_ratio < sample_ratio) ? min_ratio : sample_ratio;
    return (progress > 1.0f) ? 1.0f : progress;
}

Calib_Mag_State_t Calib_Mag_GetState(Calib_Mag_Handle_t* handle) {
    return handle->state;
}

void Calib_Mag_Apply(const Calib_Mag_t* calib,
                     const float raw[3],
                     float corrected[3]) {
    if (!calib->is_valid) {
        /* 未校准, 原样输出 */
        corrected[0] = raw[0];
        corrected[1] = raw[1];
        corrected[2] = raw[2];
        return;
    }

    /* corrected[i] = (raw[i] - bias[i]) * scale[i] */
    for (int i = 0; i < 3; i++) {
        corrected[i] = (raw[i] - calib->bias[i]) * calib->scale[i];
    }
}

void Calib_Mag_Load(Calib_Mag_Handle_t* handle, const Calib_Mag_t* calib) {
    handle->calib = *calib;
}
