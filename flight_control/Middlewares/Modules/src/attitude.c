#include "attitude.h"
#include "icm20948.h"
#include "spatial_math.h"


uint8_t imu_rx_buf[14];
uint8_t mag_rx_buf[6];
float altitude_rx;
float temperature_rx;

static sm_vec3_t gyro_current = {0};
static sm_vec3_t accel_current = {0};
static sm_vec3_t mag_current = {0};

static uint8_t accel_clib_face = 6;

static sm_quat_t last_quat;
static LowPass_Filter_t diff_angle_filter;
static LowPass_Filter_t altitude_filter;

static EKF_Handle_t imu_ekf;

static Calib_Accel_Handle_t calib_handle;
static Calib_Accel_t accel_calib;

static const float deg2rad = 0.01745329f;
static const float g = 9.80665f;
static const float still_threshold = 0.08f * deg2rad;

void Attitude_Init(sm_vec3_t accel_bias, sm_vec3_t accel_scale, float init_altitude) {
    EKF_Init(&imu_ekf);
    LowPass_Filter_Init(&diff_angle_filter, 0.1f, 0); // 初始化低通滤波器，alpha=0.1，初始输出为0
    LowPass_Filter_Init(&altitude_filter, 0.03f, init_altitude);  // 初始化高度低通滤波器，alpha=0.03，初始输出为0

    if (accel_bias[0] != 0 && accel_scale[0] != 1.0f) {
        accel_calib.is_valid = 1;
        for (int i = 0; i < 3; i++) {
            accel_calib.bias[i] = accel_bias[i];
            accel_calib.scale[i] = accel_scale[i];
        }
    } else {
        Attitude_Calibrate();
    }
}

void Attitude_Update(float dt) {
    // 通用的 FRD（前-右-下 / Forward-Right-Down）坐标系
    accel_current[0] = (int16_t)((imu_rx_buf[0] << 8) | imu_rx_buf[1]) / 2048.0f;  // X 轴 (AX)：手拿飞机，机头绝对垂直朝向天空。此时支撑力从机尾推向机头，与 X 轴同向。预期：AX 应该是正数
    accel_current[1] = (int16_t)((imu_rx_buf[2] << 8) | imu_rx_buf[3]) / 2048.0f;   // Y 轴 (AY)：手拿飞机，右侧机翼绝对垂直朝向天空（侧立）。此时支撑力从左翼推向右翼，与 Y 轴同向。预期：AY 应该是正数
    accel_current[2] = (int16_t)((imu_rx_buf[4] << 8) | imu_rx_buf[5]) / 2048.0f;  // Z 轴 (AZ)：飞机水平平放在桌面上。桌面支撑力向上，而 Z 轴指向地面（向下）。所以两者相反。预期：AZ 应该是负数

    // 1. 获取物理单位数据 (以 dps 和 g 为单位)
    gyro_current[0] = ((int16_t)((imu_rx_buf[6] << 8) | imu_rx_buf[7])) / 16.4f * deg2rad;  // X 轴 (GX / Roll)：保持水平，向右侧翻滚（右边下沉）。右手大拇指指前，四指弯向右下。预期：瞬间读数为 正(+)
    gyro_current[1] = ((int16_t)((imu_rx_buf[8] << 8) | imu_rx_buf[9])) / 16.4f * deg2rad;   // Y 轴 (GY / Pitch)：保持水平，机头向上抬（抬头）。右手大拇指指右侧，四指从下往上翻（也就是抬头）。预期：瞬间读数为 正(+)
    gyro_current[2] = ((int16_t)((imu_rx_buf[10] << 8) | imu_rx_buf[11])) / 16.4f * deg2rad;  // Z 轴 (GZ / Yaw)：保持水平，机头向右转（俯视看是顺时针转）。右手大拇指指向地面的 Z 轴，四指顺时针转。预期：瞬间读数为 正(+)

    mag_current[0] = (int16_t)((mag_rx_buf[1] << 8) | mag_rx_buf[0]) / 4096.0f;
    mag_current[1] = (int16_t)((mag_rx_buf[3] << 8) | mag_rx_buf[2]) / 4096.0f;
    mag_current[2] = (int16_t)((mag_rx_buf[5] << 8) | mag_rx_buf[4]) / 4096.0f;

    if (calib_handle.state == CALIB_ACCEL_COLLECTING && diff_angle_filter.output < still_threshold) {
        Calib_Accel_AddSample(&calib_handle, accel_current, accel_clib_face);
        if (calib_handle.state == CALIB_ACCEL_DONE) {
            accel_calib = calib_handle.calib;
            Persistence_WriteCalibData(PERSISTENCE_DATA_MARKER, accel_calib.bias, accel_calib.scale);
        }
    }

    if (accel_calib.is_valid) {
        float calibrated_accel[3];
        Calib_Accel_Apply(&accel_calib, accel_current, calibrated_accel);
        accel_current[0] = calibrated_accel[0];
        accel_current[1] = calibrated_accel[1];
        accel_current[2] = calibrated_accel[2];
    }

    // 转换到 m/s^2 交给 EKF
    float accel_ms2[3] = {accel_current[0] * g, accel_current[1] * g, accel_current[2] * g};

    LowPass_Update(&altitude_filter, altitude_rx);
    
    #ifdef ATTITUDE_EKF_BARO
    EKF_Update(&imu_ekf, accel_ms2, gyro_current, altitude_filter.output, dt);
    #else
    EKF_Update(&imu_ekf, accel_ms2, gyro_current, dt);
    #endif /* ATTITUDE_EKF_BARO */
}

void Attitude_IsStill(uint8_t *still) {
    sm_quat_t current_quat;
    memcpy(current_quat, imu_ekf.x, sizeof(sm_quat_t));

    float diff_angle = Spatial_QuatAngleBetween(current_quat, last_quat);
    LowPass_Update(&diff_angle_filter, diff_angle);
    *still = diff_angle_filter.output < still_threshold;

    if (diff_angle_filter.output > 3 * deg2rad && Calib_Accel_IsFaceDone(&calib_handle, accel_clib_face)) {
        accel_clib_face++;
    }

    memcpy(last_quat, current_quat, sizeof(sm_quat_t));
}

void Attitude_GetEuler(float *yaw, float *pitch, float *roll) {
    Spatial_QuatGetEuler(yaw, pitch, roll, imu_ekf.x);
}

void Attitude_GetQuat(sm_quat_t q) {
    memcpy(q, imu_ekf.x, sizeof(sm_quat_t));
}

void Attitude_GetGyro(sm_vec3_t gyro) {
    memcpy(gyro, gyro_current, sizeof(sm_vec3_t));
    gyro[0] -= imu_ekf.x[4];
    gyro[1] -= imu_ekf.x[5];
    gyro[2] -= imu_ekf.x[6];
}

void Attitude_GetAccel(sm_vec3_t accel) {
    // 因为内部 accel_current 保存的是 1G 的单位，这里恢复为 m/s^2 供外界(比如UI)使用
    accel[0] = accel_current[0] * g;
    accel[1] = accel_current[1] * g;
    accel[2] = accel_current[2] * g;
}

void Attitude_GetMag(sm_vec3_t mag) {
    memcpy(mag, mag_current, sizeof(sm_vec3_t));
}

void Attitude_GetAltitude(float *altitude) {
    #ifdef ATTITUDE_EKF_BARO
    *altitude = EKF_GetAltitude(&imu_ekf);
    #else
    *altitude = altitude_filter.output;
    #endif /* ATTITUDE_EKF_BARO */
}

void Attitude_GetVelocityZ(float* velocityZ) {
    #ifdef ATTITUDE_EKF_BARO
    *velocityZ = EKF_GetVelocityZ(&imu_ekf);
    #else
    *velocityZ = 0.0f;
    #endif /* ATTITUDE_EKF_BARO */
}

void Attitude_Calibrate(void) {
    accel_clib_face = 0;
    accel_calib.is_valid = 0;
    Calib_Accel_Init(&calib_handle);
    Calib_Accel_Start(&calib_handle);
}

void Attitude_CalibratingFace(uint8_t *face) {
    *face = accel_clib_face;
}