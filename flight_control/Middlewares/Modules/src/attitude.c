/**
 * @file attitude.c
 * @brief 姿态估计模块 — 基于 Error-State EKF (ekf_core)
 *
 * 调用约定 (FRD 右手定则):
 *   Roll  正 = 右倾       Gyro X 正 = 右翼下沉
 *   Pitch 正 = 抬头       Gyro Y 正 = 机头抬起
 *   Yaw   正 = 机头右偏   Gyro Z 正 = 俯视顺时针
 *   Accel 水平静止 = [0, 0, -g] m/s²
 *
 * 校准流程 (当前仅加速度计):
 *   1. Flash 无数据 → 自动启动加速度计六面校准
 *   2. 对准 (前 200 帧) → EKF 启动 → 面切换可用
 *   3. 六面完成 → 写入 Flash
 *   4. 后续上电 → 从 Flash 加载
 *
 * 磁力计: 代码保留, 当前禁用 (后续可重新启用)
 * yaw 表现: 仅靠陀螺仪积分, 会缓慢漂移 (~0.01°/s), 自稳飞行够用
 */

#include "attitude.h"
#include <math.h>
#include <string.h>
#include "calibrate_accel.h"
#include "calibrate_mag.h"
#include "ekf_core.h"
#include "lowpass.h"
#include "persistence.h"
#include "spatial_math.h"

/* ========================================================================== */
/*  配置                                                                       */
/* ========================================================================== */

/** @brief 对准所需 IMU 样本数 */
#define ALIGN_IMU_SAMPLES 3

#define ATTITUDE_EKF_BARO 1

/** @brief 是否启用磁力计 (0=禁用, 1=启用) */
#define ATTITUDE_USE_MAG 0

/* ========================================================================== */
/*  常量                                                                       */
/* ========================================================================== */

static const float deg2rad = 0.01745329f;
static const float g = 9.80665f;
static const float still_threshold = 0.08f * deg2rad; /* 静止判定门限 */

/* ========================================================================== */
/*  EKF 实例                                                                   */
/* ========================================================================== */

static ekf_t ekf;

/* ========================================================================== */
/*  对准状态机                                                                 */
/* ========================================================================== */

typedef enum {
    ATT_ALIGNING = 0, /**< 正在采集样本，未启动 EKF */
    ATT_RUNNING = 1,  /**< 对准完成，正常运行       */
} att_state_t;

static att_state_t att_state = ATT_ALIGNING;
static int align_count = 0;
static int align_mag_count = 0;
static float align_sum_ax, align_sum_ay, align_sum_az;
static float align_sum_gx, align_sum_gy, align_sum_gz;
static float align_sum_mx, align_sum_my, align_sum_mz;

/* ========================================================================== */
/*  原始数据缓冲 (由 DMA/中断写入)                                              */
/* ========================================================================== */

uint8_t imu_rx_buf[14];
uint8_t mag_rx_buf[6];
float altitude_rx;
float temperature_rx;

/* ========================================================================== */
/*  传感器数据 (校准后, 内部使用)                                                */
/* ========================================================================== */

static sm_vec3_t gyro_current = {0};
static sm_vec3_t accel_current = {0};
static sm_vec3_t mag_current = {0};

/* ========================================================================== */
/*  时间戳                                                                     */
/* ========================================================================== */

static uint64_t imu_timestamp_us = 0;

/* ========================================================================== */
/*  加速度计六面校准                                                            */
/* ========================================================================== */

static uint8_t accel_clib_face = 6; /* 6 = 未校准状态 */
static Calib_Accel_Handle_t calib_handle;
static Calib_Accel_t accel_calib;

/* ========================================================================== */
/*  磁力计校准                                                                  */
/* ========================================================================== */

static Calib_Mag_Handle_t mag_calib_handle;
static Calib_Mag_t mag_calib;
static uint8_t mag_ref_needs_update = 0;

/* ========================================================================== */
/*  滤波器                                                                     */
/* ========================================================================== */

static sm_quat_t last_quat;
static LowPass_Filter_t diff_angle_filter;
static LowPass_Filter_t altitude_filter;

/* ========================================================================== */
/*  内部: 写入 Flash                                                            */
/* ========================================================================== */

static void persistence_save(void) {
    uint32_t flags = 0;
    if (accel_calib.is_valid)
        flags |= PERSISTENCE_FLAG_ACCEL_VALID;
    if (mag_calib.is_valid)
        flags |= PERSISTENCE_FLAG_MAG_VALID;
    Persistence_WriteCalibData(PERSISTENCE_DATA_MARKER, flags,
                               accel_calib.bias, accel_calib.scale,
                               mag_calib.bias, mag_calib.scale);
}

/* ========================================================================== */
/*  内部: 磁力计校准后重算地磁参考 (当前未启用, 保留供后续使用)                  */
/* ========================================================================== */

static void update_mag_reference(void) {
#if ATTITUDE_USE_MAG
    if (!ekf.initialized)
        return;

    ekf_mat3_t Rm;
    ekf_quat_to_rotmat(&ekf.state.quat, &Rm);

    float mb[3] = {mag_current[0], mag_current[1], mag_current[2]};

    ekf.mag_ref.m_earth.x = Rm.m[0][0] * mb[0] + Rm.m[1][0] * mb[1] + Rm.m[2][0] * mb[2];
    ekf.mag_ref.m_earth.y = Rm.m[0][1] * mb[0] + Rm.m[1][1] * mb[1] + Rm.m[2][1] * mb[2];
    ekf.mag_ref.m_earth.z = Rm.m[0][2] * mb[0] + Rm.m[1][2] * mb[1] + Rm.m[2][2] * mb[2];

    float mex = ekf.mag_ref.m_earth.x;
    float mey = ekf.mag_ref.m_earth.y;
    float mez = ekf.mag_ref.m_earth.z;

    ekf.mag_ref.total_field = sqrtf(mex * mex + mey * mey + mez * mez);
    ekf.mag_ref.declination = atan2f(mey, mex);
    ekf.mag_ref.inclination = atan2f(-mez, sqrtf(mex * mex + mey * mey));
    ekf.mag_ref.calibrated = 1;
#else
    (void)0; /* 未启用 */
#endif
}

/* ========================================================================== */
/*  内部函数: 执行对准                                                          */
/* ========================================================================== */

static void attitude_try_align(void) {
    float n = (float)align_count;
    if (n < 1.0f)
        return;

    float ax = align_sum_ax / n;
    float ay = align_sum_ay / n;
    float az = align_sum_az / n;
    float gx = align_sum_gx / n;
    float gy = align_sum_gy / n;
    float gz = align_sum_gz / n;

    /* ---- Roll / Pitch (FRD) ----
     * R_w2b(φ)·[0,0,-g] = [0, -g·sin(φ), -g·cos(φ)]
     * roll  = atan2(-ay, -az)   右倾为正
     * pitch = atan2(ax, sqrt(ay²+az²))  抬头为正
     */
    float roll = atan2f(-ay, -az);
    float pitch = atan2f(ax, sqrtf(ay * ay + az * az));
    float yaw = 0.0f;

#if ATTITUDE_USE_MAG
    /* ---- Yaw + mag_ref (仅磁力计已校准时) ---- */
    if (align_mag_count > 10 && mag_calib.is_valid) {
        float mn = (float)align_mag_count;
        float mx = align_sum_mx / mn;
        float my = align_sum_my / mn;
        float mz = align_sum_mz / mn;

        float sr = sinf(roll), cr = cosf(roll);
        float sp = sinf(pitch), cp = cosf(pitch);

        float m_hx = mx * cp + my * sr * sp + mz * cr * sp;
        float m_hy = my * cr - mz * sr;
        yaw = atan2f(m_hy, m_hx);

        ekf_euler_t euler_tmp = {roll, pitch, yaw};
        ekf_quat_t q_tmp;
        ekf_euler_to_quat(&euler_tmp, &q_tmp);
        ekf_mat3_t R_tmp;
        ekf_quat_to_rotmat(&q_tmp, &R_tmp);

        float mb[3] = {mx, my, mz};
        ekf.mag_ref.m_earth.x = R_tmp.m[0][0] * mb[0] + R_tmp.m[1][0] * mb[1] + R_tmp.m[2][0] * mb[2];
        ekf.mag_ref.m_earth.y = R_tmp.m[0][1] * mb[0] + R_tmp.m[1][1] * mb[1] + R_tmp.m[2][1] * mb[2];
        ekf.mag_ref.m_earth.z = R_tmp.m[0][2] * mb[0] + R_tmp.m[1][2] * mb[1] + R_tmp.m[2][2] * mb[2];

        float mex = ekf.mag_ref.m_earth.x;
        float mey = ekf.mag_ref.m_earth.y;
        float mez = ekf.mag_ref.m_earth.z;
        ekf.mag_ref.total_field = sqrtf(mex * mex + mey * mey + mez * mez);
        ekf.mag_ref.declination = atan2f(mey, mex);
        ekf.mag_ref.inclination = atan2f(-mez, sqrtf(mex * mex + mey * mey));
        ekf.mag_ref.calibrated = 1;
    }
#endif

    /* ---- 设置 EKF 初始状态 ---- */
    ekf_state_init_default(&ekf.state);
    ekf.state.pos.z = -altitude_filter.output;

    ekf_euler_t euler_init = {roll, pitch, yaw};
    ekf_euler_to_quat(&euler_init, &ekf.state.quat);

    ekf.state.gyro_bias.x = gx;
    ekf.state.gyro_bias.y = gy;
    ekf.state.gyro_bias.z = gz;

    ekf.state.accel_bias.x = 0;
    ekf.state.accel_bias.y = 0;
    ekf.state.accel_bias.z = 0;

    ekf_cov_init_diagonal(&ekf.P, 5.0f, 0.5f, 0.1f, 0.05f, 0.3f);

    /* ---- 对准时估计 accel bias ----
     *
     * 已知: 设备静止，a_body = R_w2b·[0,0,-g] + bias
     *       R_w2b 由加速度均值估计 (roll/pitch)
     *       所以 bias = a_mean - R_w2b·[0,0,-g]
     *
     * 这是代数求解，不是 EKF 迭代，无不可观问题。
     */
    {
        float cp = cosf(pitch), sp = sinf(pitch);
        float cr = cosf(roll), sr = sinf(roll);

        /* R_w2b · [0, 0, -g] */
        float a_gravity[3] = {
            g * sp,
            -g * cp * sr,
            -g * cp * cr};

        /* bias = measured - gravity */
        ekf.state.accel_bias.x = ax - a_gravity[0];
        ekf.state.accel_bias.y = ay - a_gravity[1];
        ekf.state.accel_bias.z = az - a_gravity[2];

        /* 精度: 初始协方差小 (已从静止数据求解) */
        ekf.P.data[12][12] = 0.01f;
        ekf.P.data[13][13] = 0.01f;
        ekf.P.data[14][14] = 0.01f;
    }

    ekf.initialized = 1;
    att_state = ATT_RUNNING;
}

/* ========================================================================== */
/*  公共 API                                                                   */
/* ========================================================================== */

void Attitude_Init(sm_vec3_t accel_bias, sm_vec3_t accel_scale, float init_altitude) {
    ekf_init(&ekf, NULL);

    LowPass_Filter_Init(&diff_angle_filter, 0.1f, 0.0f);
    LowPass_Filter_Init(&altitude_filter, 0.03f, init_altitude);

    att_state = ATT_ALIGNING;
    align_count = 0;
    align_mag_count = 0;
    align_sum_ax = align_sum_ay = align_sum_az = 0;
    align_sum_gx = align_sum_gy = align_sum_gz = 0;
    align_sum_mx = align_sum_my = align_sum_mz = 0;

    imu_timestamp_us = 0;

    Calib_Mag_Init(&mag_calib_handle);
    accel_calib.is_valid = 0;
    mag_calib.is_valid = 0;
    mag_ref_needs_update = 0;

    /* ---- 从 Flash 加载校准参数 ---- */
    {
        sm_vec3_t tmp_accel_bias, tmp_accel_scale;
        float tmp_mag_bias[3] = {0}, tmp_mag_scale[3] = {1, 1, 1};
        uint32_t flags = 0;

        if (Persistence_ReadCalibData(PERSISTENCE_DATA_MARKER,
                                      &flags,
                                      tmp_accel_bias, tmp_accel_scale,
                                      tmp_mag_bias, tmp_mag_scale)) {
            if (flags & PERSISTENCE_FLAG_ACCEL_VALID) {
                for (int i = 0; i < 3; i++) {
                    accel_calib.bias[i] = tmp_accel_bias[i];
                    accel_calib.scale[i] = tmp_accel_scale[i];
                }
                accel_calib.is_valid = 1;
            }

            if (flags & PERSISTENCE_FLAG_MAG_VALID) {
                for (int i = 0; i < 3; i++) {
                    mag_calib.bias[i] = tmp_mag_bias[i];
                    mag_calib.scale[i] = tmp_mag_scale[i];
                }
                mag_calib.is_valid = 1;
            }
        } else if (accel_bias[0] != 0 && accel_scale[0] != 1.0f) {
            for (int i = 0; i < 3; i++) {
                accel_calib.bias[i] = accel_bias[i];
                accel_calib.scale[i] = accel_scale[i];
            }
            accel_calib.is_valid = 1;
        }
    }

    /* ---- 自动启动校准 ---- */
    if (!accel_calib.is_valid) {
        Attitude_Calibrate();
    }
#if ATTITUDE_USE_MAG
    else if (!mag_calib.is_valid) {
        Attitude_StartMagCalibrate();
    }
#endif
}

void Attitude_Update(float dt) {
    /* ================================================================== */
    /*  Step 1: 解析原始数据                                                */
    /* ================================================================== */

    accel_current[0] = (int16_t)((imu_rx_buf[0] << 8) | imu_rx_buf[1]) / 2048.0f;
    accel_current[1] = (int16_t)((imu_rx_buf[2] << 8) | imu_rx_buf[3]) / 2048.0f;
    accel_current[2] = (int16_t)((imu_rx_buf[4] << 8) | imu_rx_buf[5]) / 2048.0f;

    gyro_current[0] = ((int16_t)((imu_rx_buf[6] << 8) | imu_rx_buf[7])) / 16.4f * deg2rad;
    gyro_current[1] = ((int16_t)((imu_rx_buf[8] << 8) | imu_rx_buf[9])) / 16.4f * deg2rad;
    gyro_current[2] = ((int16_t)((imu_rx_buf[10] << 8) | imu_rx_buf[11])) / 16.4f * deg2rad;

    mag_current[0] = (int16_t)((mag_rx_buf[1] << 8) | mag_rx_buf[0]) * 0.02441f;
    mag_current[1] = (int16_t)((mag_rx_buf[3] << 8) | mag_rx_buf[2]) * 0.02441f;
    mag_current[2] = -(int16_t)((mag_rx_buf[5] << 8) | mag_rx_buf[4]) * 0.02441f;

    /* ================================================================== */
    /*  Step 2: 传感器校准                                                  */
    /* ================================================================== */

#if ATTITUDE_USE_MAG
    /* ---- 2a. 磁力计校准采集 (raw 数据) ---- */
    if (mag_calib_handle.state == CALIB_MAG_COLLECTING) {
        Calib_Mag_AddSample(&mag_calib_handle, mag_current);
        if (mag_calib_handle.state == CALIB_MAG_DONE) {
            mag_calib = mag_calib_handle.calib;
            persistence_save();
            mag_ref_needs_update = 1;
        }
    }

    /* ---- 2b. 应用磁力计校准 ---- */
    if (mag_calib.is_valid) {
        float mag_cal[3];
        Calib_Mag_Apply(&mag_calib, mag_current, mag_cal);
        mag_current[0] = mag_cal[0];
        mag_current[1] = mag_cal[1];
        mag_current[2] = mag_cal[2];
    }
#endif

    /* ---- 2c. 加速度计校准采集 (需要静止) ---- */
    if (calib_handle.state == CALIB_ACCEL_COLLECTING &&
        diff_angle_filter.output < still_threshold) {
        Calib_Accel_AddSample(&calib_handle, accel_current, accel_clib_face);
        if (calib_handle.state == CALIB_ACCEL_DONE) {
            accel_calib = calib_handle.calib;
            persistence_save();

#if ATTITUDE_USE_MAG
            if (!mag_calib.is_valid) {
                Attitude_StartMagCalibrate();
            }
#endif
        }
    }

    /* ---- 2d. 应用加速度计校准 ---- */
    if (accel_calib.is_valid) {
        float accel_cal[3];
        Calib_Accel_Apply(&accel_calib, accel_current, accel_cal);
        accel_current[0] = accel_cal[0];
        accel_current[1] = accel_cal[1];
        accel_current[2] = accel_cal[2];
    }

#if ATTITUDE_USE_MAG
    /* ---- 2e. 磁力计刚校准完 → 重算 mag_ref ---- */
    if (mag_ref_needs_update && ekf.initialized) {
        update_mag_reference();
        mag_ref_needs_update = 0;
    }
#endif

    /* ================================================================== */
    /*  Step 3: 单位转换                                                    */
    /* ================================================================== */

    float accel_ms2[3] = {
        accel_current[0] * g,
        accel_current[1] * g,
        accel_current[2] * g};

    float gyro_rad[3] = {
        gyro_current[0],
        gyro_current[1],
        gyro_current[2]};

    LowPass_Update(&altitude_filter, altitude_rx);
    imu_timestamp_us += (uint64_t)(dt * 1e6f);

    /* ================================================================== */
    /*  Step 4: 对准阶段                                                    */
    /* ================================================================== */

    if (att_state == ATT_ALIGNING) {
#if ATTITUDE_USE_MAG
        uint8_t mag_calibrating = (mag_calib_handle.state == CALIB_MAG_COLLECTING);
        if (!mag_calibrating) {
#else
        {
#endif
            align_sum_ax += accel_ms2[0];
            align_sum_ay += accel_ms2[1];
            align_sum_az += accel_ms2[2];
            align_sum_gx += gyro_rad[0];
            align_sum_gy += gyro_rad[1];
            align_sum_gz += gyro_rad[2];
            align_count++;

            align_sum_mx += mag_current[0];
            align_sum_my += mag_current[1];
            align_sum_mz += mag_current[2];
            align_mag_count++;

            if (align_count >= ALIGN_IMU_SAMPLES) {
                attitude_try_align();
            }
        }
        return;
    }

    /* ================================================================== */
    /*  Step 5: EKF 预测 + 量测更新                                        */
    /* ================================================================== */

    /* ---- IMU 预测 ---- */
    ekf_imu_t imu_pkt;
    imu_pkt.header.timestamp_us = imu_timestamp_us;
    imu_pkt.header.status = EKF_SENSOR_VALID;
    imu_pkt.gyro.omega_x = gyro_rad[0];
    imu_pkt.gyro.omega_y = gyro_rad[1];
    imu_pkt.gyro.omega_z = gyro_rad[2];
    imu_pkt.accel.a_x = accel_ms2[0];
    imu_pkt.accel.a_y = accel_ms2[1];
    imu_pkt.accel.a_z = accel_ms2[2];

    ekf_predict(&ekf, &imu_pkt);
    ekf_update_gravity(&ekf, &imu_pkt);

#if ATTITUDE_USE_MAG
    /* ---- 磁力计更新 (修正 yaw) ---- */
    {
        ekf_mag_t mag_pkt;
        mag_pkt.header.timestamp_us = imu_timestamp_us;
        mag_pkt.header.status = EKF_SENSOR_VALID;
        mag_pkt.m_x = mag_current[0];
        mag_pkt.m_y = mag_current[1];
        mag_pkt.m_z = mag_current[2];
        ekf_update_mag(&ekf, &mag_pkt);
    }
#endif

    /* ---- 气压计更新 (修正高度) ---- */
#ifdef ATTITUDE_EKF_BARO
    {
        ekf_baro_t baro_pkt;
        baro_pkt.header.timestamp_us = imu_timestamp_us;
        baro_pkt.header.status = EKF_SENSOR_VALID;
        baro_pkt.altitude = altitude_filter.output;
        baro_pkt.temperature = temperature_rx;
        ekf_update_baro(&ekf, &baro_pkt);
    }
#endif
}

/* ========================================================================== */

void Attitude_IsStill(uint8_t* still) {
    sm_quat_t current_quat;
    ekf_quat_t q;
    ekf_get_quat(&ekf, &q);
    current_quat[0] = q.w;
    current_quat[1] = q.x;
    current_quat[2] = q.y;
    current_quat[3] = q.z;

    float diff_angle = Spatial_QuatAngleBetween(current_quat, last_quat);
    LowPass_Update(&diff_angle_filter, diff_angle);
    *still = (diff_angle_filter.output < still_threshold);

    if (diff_angle_filter.output > 3 * deg2rad &&
        Calib_Accel_IsFaceDone(&calib_handle, accel_clib_face)) {
        accel_clib_face++;
    }

    memcpy(last_quat, current_quat, sizeof(sm_quat_t));
}

/* ========================================================================== */

void Attitude_GetEuler(float* yaw, float* pitch, float* roll) {
    ekf_euler_t euler;
    ekf_get_euler(&ekf, &euler);
    *roll = euler.roll;
    *pitch = euler.pitch;
    *yaw = euler.yaw;
}

void Attitude_GetQuat(sm_quat_t q) {
    memcpy(q, &ekf.state.quat, sizeof(sm_quat_t));
}

void Attitude_GetGyro(sm_vec3_t gyro) {
    gyro[0] = gyro_current[0] - ekf.state.gyro_bias.x;
    gyro[1] = gyro_current[1] - ekf.state.gyro_bias.y;
    gyro[2] = gyro_current[2] - ekf.state.gyro_bias.z;
}

void Attitude_GetAccel(sm_vec3_t accel) {
    accel[0] = accel_current[0] * g;
    accel[1] = accel_current[1] * g;
    accel[2] = accel_current[2] * g;
}

void Attitude_GetMag(sm_vec3_t mag) {
    memcpy(mag, mag_current, sizeof(sm_vec3_t));
}

void Attitude_GetAltitude(float* altitude) {
#ifdef ATTITUDE_EKF_BARO
    *altitude = -ekf.state.pos.z;
#else
    *altitude = altitude_filter.output;
#endif
}

void Attitude_GetVelocityZ(float* velocityZ) {
#ifdef ATTITUDE_EKF_BARO
    *velocityZ = -ekf.state.vel.z;
#else
    *velocityZ = 0.0f;
#endif
}

void Attitude_Calibrate(void) {
    accel_clib_face = 0;
    accel_calib.is_valid = 0;
    Calib_Accel_Init(&calib_handle);
    Calib_Accel_Start(&calib_handle);
}

void Attitude_CalibratingFace(uint8_t* face) {
    *face = accel_clib_face;
}

void Attitude_StartMagCalibrate(void) {
    Calib_Mag_Start(&mag_calib_handle);
}

float Attitude_MagCalibrateProgress(void) {
    return Calib_Mag_GetProgress(&mag_calib_handle);
}

uint8_t Attitude_MagCalibStatus(void) {
    return (uint8_t)mag_calib_handle.state;
}

uint8_t Attitude_MagCalibValid(void) {
    return mag_calib.is_valid;
}
