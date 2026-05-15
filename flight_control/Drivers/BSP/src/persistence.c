#include "persistence.h"

static uint32_t calc_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}

void Persistence_WriteCalibData(uint32_t marker, const sm_vec3_t accel_bias, const sm_vec3_t accel_scale) {
    FLASH_Unlock();
    FLASH_EraseSector(FLASH_Sector_5, VoltageRange_3);

    FLASH_ProgramWord(FLASH_BIAS_ADDR + 0 * 4, marker);
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 1 * 4, *((uint32_t*)&accel_bias[0]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 2 * 4, *((uint32_t*)&accel_bias[1]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 3 * 4, *((uint32_t*)&accel_bias[2]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 4 * 4, *((uint32_t*)&accel_scale[0]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 5 * 4, *((uint32_t*)&accel_scale[1]));
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 6 * 4, *((uint32_t*)&accel_scale[2]));

    uint32_t crc = calc_crc32((const uint8_t*)FLASH_BIAS_ADDR, 7 * 4);
    FLASH_ProgramWord(FLASH_BIAS_ADDR + 7 * 4, crc);

    FLASH_Lock();
}

uint8_t Persistence_ReadCalibData(uint32_t marker, sm_vec3_t accel_bias, sm_vec3_t accel_scale) {
    uint32_t* udata = (uint32_t*)FLASH_BIAS_ADDR;

    if (udata[0] == marker) {
        uint32_t expected_crc = calc_crc32((const uint8_t*)FLASH_BIAS_ADDR, 7 * 4);
        if (udata[7] != expected_crc)
            return 0;

        memcpy(accel_bias, &udata[1], 3 * sizeof(float));
        memcpy(accel_scale, &udata[4], 3 * sizeof(float));
        return 1;
    }

    return 0;
}
