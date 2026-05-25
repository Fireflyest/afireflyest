#ifndef __EXTRA_SENSOR_H
#define __EXTRA_SENSOR_H

#include <string.h>
#include "stm32f4xx.h"

#define ES_BAUDRATE_9600 ((uint32_t)9600)
#define ES_BAUDRATE_115200 ((uint32_t)115200)

#define ES_RX_BUFFER_SIZE 64
#define ES_TX_BUFFER_SIZE 64

#define ES_RX_STATE_IDLE 0
#define ES_RX_STATE_COMPLETE 1

extern uint8_t esRxBuffer[ES_RX_BUFFER_SIZE];
extern uint8_t esTxBuffer[ES_TX_BUFFER_SIZE];

extern __IO uint16_t esRxIndexUart2;
extern __IO uint8_t esRxStatusUart2;

void ExtraSensor_Init(uint32_t baudrate);

uint16_t ExtraSensor_ReadData(uint8_t* data);
void ExtraSensor_HandleCommand(void);

void ExtraSensor_WriteData(uint8_t* data, uint16_t size);

#endif /* __EXTRA_SENSOR_H */