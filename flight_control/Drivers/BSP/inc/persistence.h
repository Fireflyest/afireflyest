#ifndef __PERSISTENCE_H
#define __PERSISTENCE_H

#include <string.h>
#include "spatial_math.h"
#include "stm32f4xx.h"

#define PERSISTENCE_DATA_MARKER 0xA5A5A5A5
#define FLASH_BIAS_ADDR ((uint32_t)0x0803E000)

/**
 * Flash 布局 (16 words = 64 bytes):
 *
 *   +0   [1 word]  marker
 *   +4   [1 word]  flags (bit0: accel有效, bit1: mag有效)
 *   +8   [3 words] accel_bias[3]
 *   +20  [3 words] accel_scale[3]
 *   +32  [3 words] mag_bias[3]
 *   +44  [3 words] mag_scale[3]
 *   +56  [1 word]  crc32 (覆盖 bytes 0~55)
 */

#define PERSISTENCE_FLAG_ACCEL_VALID (1 << 0)
#define PERSISTENCE_FLAG_MAG_VALID (1 << 1)

void Persistence_WriteCalibData(uint32_t marker,
                                uint32_t flags,
                                const sm_vec3_t accel_bias,
                                const sm_vec3_t accel_scale,
                                const float mag_bias[3],
                                const float mag_scale[3]);

uint8_t Persistence_ReadCalibData(uint32_t marker,
                                  uint32_t* flags,
                                  sm_vec3_t accel_bias,
                                  sm_vec3_t accel_scale,
                                  float mag_bias[3],
                                  float mag_scale[3]);

#endif /* __PERSISTENCE_H */
