#include "spi_sensor.h"
#include <stddef.h>

static SPI_Sensor_HandleTypeDef spi_sensors[SPI_SENSORS_MAX] = {{0}};
static uint8_t spi_sensor_count = 0;

void SPI_Sensor_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    SPI_I2S_DeInit(SPI_SENSOR);

    SPI_InitTypeDef SPI_InitStructure;
    /* SPI 配置 - Mode 0 (CPOL=0, CPHA=0) */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPI_SENSOR, &SPI_InitStructure);
    SPI_Cmd(SPI_SENSOR, ENABLE);
}

uint8_t SPI_Sensor_Register(GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    if (spi_sensor_count < SPI_SENSORS_MAX) {
        spi_sensors[spi_sensor_count].cs_port = cs_port;
        spi_sensors[spi_sensor_count].cs_pin = cs_pin;
        spi_sensor_count++;
        return spi_sensor_count;
    }
    return 0;
}


void SPI_Sensor_On(uint8_t sensor_id) {
    if (sensor_id > 0 && sensor_id <= spi_sensor_count) {
        GPIO_ResetBits(spi_sensors[sensor_id - 1].cs_port, spi_sensors[sensor_id - 1].cs_pin);
    }
}

void SPI_Sensor_Off(uint8_t sensor_id) {
    if (sensor_id > 0 && sensor_id <= spi_sensor_count) {
        GPIO_SetBits(spi_sensors[sensor_id - 1].cs_port, spi_sensors[sensor_id - 1].cs_pin);
    }
}


uint8_t SPI_Sensor_TransferByte(uint8_t tx) {
    uint16_t timeout;
    for (timeout = 0x5FFF; timeout > 0 && SPI_I2S_GetFlagStatus(SPI_SENSOR, SPI_I2S_FLAG_TXE) == RESET; timeout--);
    SPI_I2S_SendData(SPI_SENSOR, tx);
    for (timeout = 0x5FFF; timeout > 0 && SPI_I2S_GetFlagStatus(SPI_SENSOR, SPI_I2S_FLAG_RXNE) == RESET; timeout--);
    return SPI_I2S_ReceiveData(SPI_SENSOR);
}