#ifndef __BMP280_H
#define __BMP280_H

#include "stm32f4xx.h"
#include "board.h"
#include "spi_sensor.h"
#include "math.h"

#define BMP_CTRL_MEAS      0xF4
#define BMP_CONFIG         0xF5

#define BMP_SEA_LEVEL_PRESSURE_HPA    1013.25f
#define BMP_BAROMETERIC_PRESSURE_OFFSET  -2.4f  // in hPa

void BMP280_Init(void);
uint8_t BMP280_Read_WhoAmI(void);
void BMP280_Read(float* alt, float *tmp);

#endif /* __BMP280_H */