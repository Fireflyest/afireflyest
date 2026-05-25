#ifndef __SPI_SENSOR_H
#define __SPI_SENSOR_H

#include "stm32f4xx.h"
#include "board.h"


#define SPI_SENSORS_MAX 3

typedef struct {
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} SPI_Sensor_HandleTypeDef;


/**
 * @brief Initialize the GPIO pins for SPI2
 * SPI2: SCK (PB13), MISO (PB14), MOSI (PB15)
 * ICM-20948 CS (PB12), BMP280 CS (PA5)
 */
void SPI_Sensor_Init(void);

/**
 * @brief Register a new SPI sensor with its CS pin
 * 
 * @param cs_port cs pin GPIO port
 * @param cs_pin cs pin GPIO pin number
 * @return uint8_t sensor ID (1-based), or 0 if registration failed (max 3 sensors)
 */
uint8_t SPI_Sensor_Register(GPIO_TypeDef *cs_port, uint16_t cs_pin);


void SPI_Sensor_On(uint8_t sensor_id);
void SPI_Sensor_Off(uint8_t sensor_id);

uint8_t SPI_Sensor_TransferByte(uint8_t tx);

int SPI_Sensor_ReadBytes(uint8_t sensor_id, uint8_t reg, uint8_t* data, uint16_t len);
int SPI_Sensor_WriteBytes(uint8_t sensor_id, uint8_t reg, const uint8_t* data, uint16_t len);

#endif /* __SPI_SENSOR_H */