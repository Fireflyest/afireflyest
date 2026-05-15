#ifndef __PERSISTENCE_H
#define __PERSISTENCE_H

#include "stm32f4xx.h"
#include "spatial_math.h"
#include <string.h>

#define PERSISTENCE_DATA_MARKER 0xA5A5A5A5
#define FLASH_BIAS_ADDR  ((uint32_t)0x0803E000)

void Persistence_WriteCalibData(uint32_t marker, const sm_vec3_t accel_bias, const sm_vec3_t accel_scale);
uint8_t Persistence_ReadCalibData(uint32_t marker, sm_vec3_t accel_bias, sm_vec3_t accel_scale);

#endif /* __PERSISTENCE_H */