#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "calibrate_accel.h"
#include "lowpass.h"
#include "persistence.h"
#include "spatial_math.h"
#include "stm32f4xx.h"

#define ATTITUDE_EKF_BARO 1

#ifdef ATTITUDE_EKF_BARO
#include "ekf_state7.h"
#else
#include "ekf_state6.h"
#endif /* ATTITUDE_EKF_BARO */

extern uint8_t imu_rx_buf[14];
extern uint8_t mag_rx_buf[6];
extern float altitude_rx;
extern float temperature_rx;
// extern uint8_t bmp_rx_buf[6]

void Attitude_Init(sm_vec3_t accel_bias, sm_vec3_t accel_scale, float init_altitude);
void Attitude_Update(float dt);
void Attitude_IsStill(uint8_t *still);
void Attitude_GetEuler(float *yaw, float *pitch, float *roll);
void Attitude_GetQuat(sm_quat_t q);
void Attitude_GetGyro(sm_vec3_t gyro);
void Attitude_GetAccel(sm_vec3_t accel);
// void Attitude_GetMag(sm_vec3_t mag);
void Attitude_GetAltitude(float *altitude);
void Attitude_GetVelocityZ(float *velocityZ);

void Attitude_Calibrate(void);
void Attitude_CalibratingFace(uint8_t *face);


#endif /* __ATTITUDE_H */