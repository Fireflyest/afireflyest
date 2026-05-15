#ifndef __BLE_H
#define __BLE_H

#include "stm32f4xx.h"
#include <string.h>

#define BLE_BAUDRATE_9600      ((uint32_t)9600)
#define BLE_BAUDRATE_115200    ((uint32_t)115200)

#define BLE_RX_BUFFER_SIZE     64
#define BLE_TX_BUFFER_SIZE     64

#define BLE_RX_STATE_IDLE      0
#define BLE_RX_STATE_COMPLETE  1

extern uint8_t bleRxBuffer[BLE_RX_BUFFER_SIZE];
extern uint8_t bleTxBuffer[BLE_TX_BUFFER_SIZE];

extern __IO uint16_t bleRxIndexUart1;
extern __IO uint8_t bleRxStatusUart1;

void BLE_Init(uint32_t baudrate);

uint16_t BLE_ReadData(uint8_t* data);
void BLE_HandleCommand(void);


void BLE_WriteData(uint8_t* data, uint16_t size);

#endif /* __BLE_H */