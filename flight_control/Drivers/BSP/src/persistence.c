#include "persistence.h"

#define CALIB_WORDS 15 /* marker + flags + 3 accel_bias + 3 accel_scale + 3 mag_bias + 3 mag_scale + crc */

static uint32_t calc_crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}

void Persistence_WriteCalibData(uint32_t marker,
                                uint32_t flags,
                                const sm_vec3_t accel_bias,
                                const sm_vec3_t accel_scale,
                                const float mag_bias[3],
                                const float mag_scale[3]) {
    FLASH_Unlock();
    FLASH_EraseSector(FLASH_Sector_5, VoltageRange_3);

    /* word 0: marker */
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 0 * 4, marker);
    /* word 1: flags */
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 1 * 4, flags);

    /* words 2-4: accel bias */
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 2 * 4, *((uint32_t*)&accel_bias[0]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 3 * 4, *((uint32_t*)&accel_bias[1]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 4 * 4, *((uint32_t*)&accel_bias[2]));

    /* words 5-7: accel scale */
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 5 * 4, *((uint32_t*)&accel_scale[0]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 6 * 4, *((uint32_t*)&accel_scale[1]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 7 * 4, *((uint32_t*)&accel_scale[2]));

    /* words 8-10: mag bias */
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 8 * 4, *((uint32_t*)&mag_bias[0]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 9 * 4, *((uint32_t*)&mag_bias[1]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 10 * 4, *((uint32_t*)&mag_bias[2]));

    /* words 11-13: mag scale */
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 11 * 4, *((uint32_t*)&mag_scale[0]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 12 * 4, *((uint32_t*)&mag_scale[1]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 13 * 4, *((uint32_t*)&mag_scale[2]));

    /* word 14: CRC */
    uint32_t crc = calc_crc32((const uint8_t*)FLASH_BIAS_ADDR, (CALIB_WORDS - 1) * 4);
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 14 * 4, crc);

    FLASH_Lock();
}

uint8_t Persistence_ReadCalibData(uint32_t marker,
                                  uint32_t* flags,
                                  sm_vec3_t accel_bias,
                                  sm_vec3_t accel_scale,
                                  float mag_bias[3],
                                  float mag_scale[3]) {
    uint32_t* udata = (uint32_t*)FLASH_BIAS_ADDR;

    if (udata[0] != marker)
        return 0;

    uint32_t expected_crc = calc_crc32((const uint8_t*)FLASH_BIAS_ADDR,
                                       (CALIB_WORDS - 1) * 4);
    if (udata[14] != expected_crc)
        return 0;

    *flags = udata[1];

    memcpy(accel_bias, &udata[2], 3 * sizeof(float));
    memcpy(accel_scale, &udata[5], 3 * sizeof(float));
    memcpy(mag_bias, &udata[8], 3 * sizeof(float));
    memcpy(mag_scale, &udata[11], 3 * sizeof(float));

    return 1;
}
