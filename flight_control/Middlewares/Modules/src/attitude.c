/**
 * @file attitude.c
 * @brief 姿态估计模块 — 基于 Error-State EKF (ekf_core)
 *
 * 与旧版 EKF 的主要区别:
 *   - 15 维误差状态 (位置/速度/姿态/bias)，而非仅 7 维 (四元数+bias)
 *   - 磁力计融合修正 yaw (旧版未使用磁力计)
 *   - 气压计通过独立量测更新融合 (旧版混在单次 Update 中)
 *   - 静止对准: 启动时 0.5 秒对准，估计初始姿态和 gyro bias
 *   - 欧拉角提取使用标准 ZYX 旋转顺序
 *
 * 调用约定 (FRD 右手定则):
 *   Roll  正 = 右倾       Gyro X 正 = 右翼下沉
 *   Pitch 正 = 抬头       Gyro Y 正 = 机头抬起
 *   Yaw   正 = 机头右偏   Gyro Z 正 = 俯视顺时针
 *   Accel 水平静止 = [0, 0, -g] m/s²
 *
 * 校准流程:
 *   1. Flash 无数据 → 自动启动加速度计六面校准
 *   2. 加速度计完成 → 自动启动磁力计旋转校准
 *   3. 磁力计完成 → 两套参数一起写入 Flash (带 flags 标记)
 *   4. 后续上电 → 从 Flash 加载，flags 确认各自有效性
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

/** @brief 对准所需 IMU 样本数 (400Hz 时约 0.5 秒) */
#define ALIGN_IMU_SAMPLES 200

#define ATTITUDE_EKF_BARO 1
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
/* IMU 累加和 (加速度 m/s², 角速度 rad/s) */
static float align_sum_ax, align_sum_ay, align_sum_az;
static float align_sum_gx, align_sum_gy, align_sum_gz;
/* 磁力计累加和 (μT) */
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

static sm_vec3_t gyro_current = {0};  /* rad/s, raw (未补偿 bias) */
static sm_vec3_t accel_current = {0}; /* g 单位, 校准后           */
static sm_vec3_t mag_current = {0};   /* μT, 校准后 (或 raw)      */

static float g_init_altitude = 0;

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
/*  内部: 写入 Flash (加速度计 + 磁力计, 带 flags)                              */
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
/*  内部函数: 执行对准                                                          */
/* ========================================================================== */

/**
 * @brief 对准完成回调 — 由对准状态机在采集足够样本后调用
 *
 * 执行:
 *   1. 由加速度计均值估计 roll, pitch
 *   2. 由磁力计均值估计 yaw (若可用)
 *   3. 设置 EKF 初始四元数、gyro bias、协方差
 *   4. 计算并存储地磁参考向量 (供后续 mag update 使用)
 *   5. 切换到 ATT_RUNNING
 */
static void attitude_try_align(void) {
    float n = (float)align_count;
    if (n < 1.0f)
        return;

    /* ---- 计算均值 ---- */
    float ax = align_sum_ax / n;
    float ay = align_sum_ay / n;
    float az = align_sum_az / n;
    float gx = align_sum_gx / n;
    float gy = align_sum_gy / n;
    float gz = align_sum_gz / n;

    /* ---- 由加速度计估计 Roll / Pitch ---- */
    /*
     * 加速度计测量比力 (specific force):
     *   水平静止时 a_body = [0, 0, -g]
     *
     * Roll  = atan2(a_y, -a_z)         右倾为正
     * Pitch = atan2(-a_x, sqrt(ay²+az²)) 抬头为正
     *
     * 推导 (ZYX, R = Rz·Ry·Rx, f_body = R·[0,0,-g]):
     *   水平: a = [0, 0, -g]      → roll=0, pitch=0
     *   右倾 φ: a = [0, g·sinφ, -g·cosφ] → roll = φ
     *   抬头 θ: a = [-g·sinθ, 0, -g·cosθ] → pitch = θ (注意 -ax)
     */
    float roll = atan2f(-ay, -az);
    float pitch = atan2f(ax, sqrtf(ay * ay + az * az));
    float yaw = 0.0f;

    /* ---- 由磁力计估计 Yaw ---- */
    if (align_mag_count > 10 && mag_calib.is_valid) {
        float mn = (float)align_mag_count;
        float mx = align_sum_mx / mn;
        float my = align_sum_my / mn;
        float mz = align_sum_mz / mn;

        /* 倾斜补偿: 将 body 系磁力计投影到水平面 */
        float sr = sinf(roll), cr = cosf(roll);
        float sp = sinf(pitch), cp = cosf(pitch);

        /*  m_h = Rx(-φ)·Ry(-θ)·m_body 的前两个分量
         *  m_hx = mx·cosθ + my·sinθ·sinφ + mz·sinθ·cosφ
         *  m_hy = my·cosφ - mz·sinφ
         */
        float m_hx = mx * cp + my * sr * sp + mz * cr * sp;
        float m_hy = my * cr - mz * sr;

        /* 航向: atan2(m_hy, m_hx)
         * 推导: m_body_h = Rz(ψ)·[mN, mE]^T
         *   当 mE≈0 时: m_hx=mN·cosψ, m_hy=mN·sinψ
         *   → ψ = atan2(m_hy, m_hx), 正值 = 机头右偏
         */
        yaw = atan2f(m_hy, m_hx);

        /* ---- 存储地磁参考 (供 ekf_update_mag 使用) ---- */
        /* m_earth = R_init^T · m_body */
        ekf_euler_t euler_tmp = {roll, pitch, yaw};
        ekf_quat_t q_tmp;
        ekf_euler_to_quat(&euler_tmp, &q_tmp);

        ekf_mat3_t R_tmp;
        ekf_quat_to_rotmat(&q_tmp, &R_tmp);

        /* R^T · m_body: R^T[i][j] = R[j][i] */
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

    /* ---- 设置 EKF 初始状态 ---- */
    ekf_state_init_default(&ekf.state);

    ekf.state.pos.z = -altitude_filter.output;

    /* 姿态 */
    ekf_euler_t euler_init = {roll, pitch, yaw};
    ekf_euler_to_quat(&euler_init, &ekf.state.quat);

    /* Gyro bias: 静止时角速度均值即为 bias */
    ekf.state.gyro_bias.x = gx;
    ekf.state.gyro_bias.y = gy;
    ekf.state.gyro_bias.z = gz;

    /* Accel bias: 保持为零 (已由外部校准补偿, EKF 残差估计) */
    ekf.state.accel_bias.x = 0;
    ekf.state.accel_bias.y = 0;
    ekf.state.accel_bias.z = 0;

    /* 初始协方差 (对准后比默认值小) */
    ekf_cov_init_diagonal(&ekf.P,
                          5.0f,  /* 位置: 5 m          */
                          0.5f,  /* 速度: 0.5 m/s      */
                          0.1f,  /* 姿态: 0.1 rad (~6°) */
                          0.05f, /* gyro bias: 0.05 rad/s */
                          0.3f); /* accel bias: 0.3 m/s²  */

    ekf.initialized = 1;
    att_state = ATT_RUNNING;
}

static void update_mag_reference(void) {
    if (!ekf.initialized)
        return;

    ekf_mat3_t Rm;
    ekf_quat_to_rotmat(&ekf.state.quat, &Rm);

    float mb[3] = {mag_current[0], mag_current[1], mag_current[2]};

    /* m_earth = R_b2w · m_body = Rm^T · m_body */
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
}

/* ========================================================================== */
/*  公共 API                                                                   */
/* ========================================================================== */

void Attitude_Init(sm_vec3_t accel_bias, sm_vec3_t accel_scale, float init_altitude) {
    /* ---- 初始化 EKF ---- */
    ekf_init(&ekf, NULL);
    g_init_altitude = init_altitude; /* [FIX] 保存, 等对准时使用 */

    /* ---- 初始化滤波器 ---- */
    LowPass_Filter_Init(&diff_angle_filter, 0.1f, 0.0f);
    LowPass_Filter_Init(&altitude_filter, 0.03f, init_altitude);

    /* ---- 重置对准状态 ---- */
    att_state = ATT_ALIGNING;
    align_count = 0;
    align_mag_count = 0;
    align_sum_ax = align_sum_ay = align_sum_az = 0;
    align_sum_gx = align_sum_gy = align_sum_gz = 0;
    align_sum_mx = align_sum_my = align_sum_mz = 0;

    /* ---- 重置时间戳 ---- */
    imu_timestamp_us = 0;

    /* ---- 初始化校准句柄 ---- */
    Calib_Mag_Init(&mag_calib_handle);
    accel_calib.is_valid = 0;
    mag_calib.is_valid = 0;
    mag_ref_needs_update = 0;

    /* ---- 从 Flash 加载校准参数 (加速度计 + 磁力计) ---- */
    {
        sm_vec3_t tmp_accel_bias, tmp_accel_scale;
        float tmp_mag_bias[3] = {0}, tmp_mag_scale[3] = {1, 1, 1};
        uint32_t flags = 0;

        if (Persistence_ReadCalibData(PERSISTENCE_DATA_MARKER,
                                      &flags,
                                      tmp_accel_bias, tmp_accel_scale,
                                      tmp_mag_bias, tmp_mag_scale)) {
            /* Flash 校验通过 → 按 flags 标记分别加载 */
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
        }
        /* Flash 无数据 → 尝试外部传入的加速度计参数 */
        else if (accel_bias[0] != 0 && accel_scale[0] != 1.0f) {
            for (int i = 0; i < 3; i++) {
                accel_calib.bias[i] = accel_bias[i];
                accel_calib.scale[i] = accel_scale[i];
            }
            accel_calib.is_valid = 1;
        }
        /* 都没有 → 启动六面校准 */
        else {
            Attitude_Calibrate();
        }
    }
}

void Attitude_Update(float dt) {
    /* ================================================================== */
    /*  Step 1: 解析原始数据                                                */
    /* ================================================================== */

    /* 加速度计: ±16g 量程, 2048 LSB/g → 单位 g
     *
     * FRD 约定验证:
     *   X (AX): 机头朝天, 支撑力沿 +X → 正
     *   Y (AY): 右翼朝天, 支撑力沿 +Y → 正
     *   Z (AZ): 水平静止, 支撑力向上与 +Z(下) 相反 → 负
     */
    accel_current[0] = (int16_t)((imu_rx_buf[0] << 8) | imu_rx_buf[1]) / 2048.0f;
    accel_current[1] = (int16_t)((imu_rx_buf[2] << 8) | imu_rx_buf[3]) / 2048.0f;
    accel_current[2] = (int16_t)((imu_rx_buf[4] << 8) | imu_rx_buf[5]) / 2048.0f;

    /* 陀螺仪: ±2000dps 量程, 16.4 LSB/dps → rad/s
     *
     * FRD 约定验证:
     *   GX (Roll):  右翼下沉 → 正
     *   GY (Pitch): 机头抬起 → 正
     *   GZ (Yaw):   俯视顺时针 → 正
     */
    gyro_current[0] = ((int16_t)((imu_rx_buf[6] << 8) | imu_rx_buf[7])) / 16.4f * deg2rad;
    gyro_current[1] = ((int16_t)((imu_rx_buf[8] << 8) | imu_rx_buf[9])) / 16.4f * deg2rad;
    gyro_current[2] = ((int16_t)((imu_rx_buf[10] << 8) | imu_rx_buf[11])) / 16.4f * deg2rad;

    /* 磁力计: MMC5983MA, ±800μT, 16-bit 有符号
     * 灵敏度: 800μT / 32768 = 0.02441 μT/LSB
     *
     * 注意:
     *   1. 字节序为小端 [buf[1]:buf[0]]，与 IMU 大端不同
     *   2. Z 轴与 FRD 相反，需取反 (传感器 Z 轴朝上)
     *
     * FRD 约定验证 (m_earth = [m_N, m_E, m_D] 为当地 NED 地磁参考):
     *   MX: 水平朝北静止 → X轴朝北 → 测得 m_N → 正 (北半球约 +25~35 μT)
     *   MY: 水平朝东静止 → Y轴朝东 → 测得 m_E → 正或小值
     *   MZ: 水平静止     → Z轴朝下 → 测得 m_D → 正 (北半球磁倾角向下)
     *
     * 旋转验证:
     *   水平转一圈, MX 应在 ±m_N 间振荡 (约 ±30 μT)
     *   总强度 sqrt(MX²+MY²+MZ²) 应恒定 (约 45~60 μT)
     */
    mag_current[0] = (int16_t)((mag_rx_buf[1] << 8) | mag_rx_buf[0]) * 0.02441f;
    mag_current[1] = (int16_t)((mag_rx_buf[3] << 8) | mag_rx_buf[2]) * 0.02441f;
    mag_current[2] = -(int16_t)((mag_rx_buf[5] << 8) | mag_rx_buf[4]) * 0.02441f;

    /* ================================================================== */
    /*  Step 2: 传感器校准                                                  */
    /* ================================================================== */

    /* ---- 2a. 磁力计校准采集 (使用校准前的 raw 数据) ---- */
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

    /* ---- 2c. 加速度计校准采集 (需要静止) ---- */
    if (calib_handle.state == CALIB_ACCEL_COLLECTING &&
        diff_angle_filter.output < still_threshold) {
        Calib_Accel_AddSample(&calib_handle, accel_current, accel_clib_face);
        if (calib_handle.state == CALIB_ACCEL_DONE) {
            accel_calib = calib_handle.calib;
            persistence_save();

            /* 加速度计校准完成 → 自动启动磁力计校准 (如果尚未校准) */
            if (!mag_calib.is_valid) {
                Calib_Mag_Start(&mag_calib_handle);
            }
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

    if (mag_ref_needs_update && ekf.initialized) {
        /* 此时 mag_current 已经过 Calib_Mag_Apply 校准 (Step 2b)
         * 且 EKF 姿态基本正确 (仅 yaw 可能偏)
         * 用当前姿态 + 校准后 mag 重建参考 */
        update_mag_reference();
        mag_ref_needs_update = 0;
    }

    /* ================================================================== */
    /*  Step 3: 单位转换                                                    */
    /* ================================================================== */

    /* 加速度: g → m/s² */
    float accel_ms2[3] = {
        accel_current[0] * g,
        accel_current[1] * g,
        accel_current[2] * g};

    /* 陀螺仪: 已经是 rad/s */
    float gyro_rad[3] = {
        gyro_current[0],
        gyro_current[1],
        gyro_current[2]};

    /* 气压计: 低通滤波 */
    LowPass_Update(&altitude_filter, altitude_rx);

    /* 更新时间戳 */
    imu_timestamp_us += (uint64_t)(dt * 1e6f);

    /* ================================================================== */
    /*  Step 4: 对准阶段 — 采集样本，不运行 EKF                              */
    /* ================================================================== */

    if (att_state == ATT_ALIGNING) {
        /* 累加 IMU 样本 */
        align_sum_ax += accel_ms2[0];
        align_sum_ay += accel_ms2[1];
        align_sum_az += accel_ms2[2];
        align_sum_gx += gyro_rad[0];
        align_sum_gy += gyro_rad[1];
        align_sum_gz += gyro_rad[2];
        align_count++;

        /* 累加磁力计样本 (可能与 IMU 不同步，单独计数) */
        align_sum_mx += mag_current[0];
        align_sum_my += mag_current[1];
        align_sum_mz += mag_current[2];
        align_mag_count++;

        /* 样本足够 → 执行对准 */
        if (align_count >= ALIGN_IMU_SAMPLES) {
            attitude_try_align();
        }
        return; /* 对准期间不运行 EKF，保持默认状态 */
    }

    /* ================================================================== */
    /*  Step 5: 正常运行 — EKF 预测 + 量测更新                              */
    /* ================================================================== */

    /* ---- 构建 IMU 数据包 ---- */
    ekf_imu_t imu_pkt;
    imu_pkt.header.timestamp_us = imu_timestamp_us;
    imu_pkt.header.status = EKF_SENSOR_VALID;

    imu_pkt.gyro.header = imu_pkt.header;
    imu_pkt.gyro.omega_x = gyro_rad[0];
    imu_pkt.gyro.omega_y = gyro_rad[1];
    imu_pkt.gyro.omega_z = gyro_rad[2];

    imu_pkt.accel.header = imu_pkt.header;
    imu_pkt.accel.a_x = accel_ms2[0];
    imu_pkt.accel.a_y = accel_ms2[1];
    imu_pkt.accel.a_z = accel_ms2[2];

    /* ---- 预测 (每次 IMU 数据都调用) ---- */
    ekf_predict(&ekf, &imu_pkt);

    /* ---- 磁力计更新 (修正 yaw) ---- */
    ekf_mag_t mag_pkt;
    mag_pkt.header.timestamp_us = imu_timestamp_us;
    mag_pkt.header.status = EKF_SENSOR_VALID;
    mag_pkt.m_x = mag_current[0];
    mag_pkt.m_y = mag_current[1];
    mag_pkt.m_z = mag_current[2];

    ekf_update_mag(&ekf, &mag_pkt);

    /* ---- 气压计更新 (修正高度, 条件编译) ---- */
#ifdef ATTITUDE_EKF_BARO
    ekf_baro_t baro_pkt;
    baro_pkt.header.timestamp_us = imu_timestamp_us;
    baro_pkt.header.status = EKF_SENSOR_VALID;
    baro_pkt.altitude = altitude_filter.output; /* m, 向上为正 */
    baro_pkt.temperature = temperature_rx;

    ekf_update_baro(&ekf, &baro_pkt);
#endif
}

/* ========================================================================== */

void Attitude_IsStill(uint8_t* still) {
    /* 从 EKF 获取当前四元数 */
    sm_quat_t current_quat;
    ekf_quat_t q;
    ekf_get_quat(&ekf, &q);
    current_quat[0] = q.w;
    current_quat[1] = q.x;
    current_quat[2] = q.y;
    current_quat[3] = q.z;

    /* 计算与上次的旋转角度差 */
    float diff_angle = Spatial_QuatAngleBetween(current_quat, last_quat);
    LowPass_Update(&diff_angle_filter, diff_angle);
    *still = (diff_angle_filter.output < still_threshold);

    /* 校准流程: 运动足够大且当前面已完成 → 切换到下一个面 */
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

    *roll = euler.roll;   /* rad, 右倾为正  */
    *pitch = euler.pitch; /* rad, 抬头为正  */
    *yaw = euler.yaw;     /* rad, 右偏为正  */
}

void Attitude_GetQuat(sm_quat_t q) {
    /* ekf_quat_t {w,x,y,z} 内存布局 与 sm_quat_t [4] 一致 (均为 float, 无 padding) */
    memcpy(q, &ekf.state.quat, sizeof(sm_quat_t));
}

void Attitude_GetGyro(sm_vec3_t gyro) {
    /* 返回 bias 补偿后的陀螺仪角速度 (rad/s) */
    gyro[0] = gyro_current[0] - ekf.state.gyro_bias.x;
    gyro[1] = gyro_current[1] - ekf.state.gyro_bias.y;
    gyro[2] = gyro_current[2] - ekf.state.gyro_bias.z;
}

void Attitude_GetAccel(sm_vec3_t accel) {
    /* 返回 m/s² (校准后, 已包含重力分量) */
    accel[0] = accel_current[0] * g;
    accel[1] = accel_current[1] * g;
    accel[2] = accel_current[2] * g;
}

void Attitude_GetMag(sm_vec3_t mag) {
    memcpy(mag, mag_current, sizeof(sm_vec3_t));
}

void Attitude_GetAltitude(float* altitude) {
#ifdef ATTITUDE_EKF_BARO
    /* NED: pos.z 向下为正; 高度: 向上为正 → 取负 */
    *altitude = -ekf.state.pos.z;
#else
    *altitude = altitude_filter.output;
#endif
}

void Attitude_GetVelocityZ(float* velocityZ) {
#ifdef ATTITUDE_EKF_BARO
    /* NED: vel.z 向下为正; 垂直速度: 向上为正 → 取负 */
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