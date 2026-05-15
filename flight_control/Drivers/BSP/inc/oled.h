#ifndef __I2COLED_H
#define __I2COLED_H

#include "stm32f4xx.h"

// OLED Parameters
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

#define COMMUICATION_TYPE_I2C
// #define COMMUNICATION_TYPE_SPI

extern uint16_t hasOLED;

// Hardware Initialization
void OLED_Init(void);

// Screen Update Interface
// Sends a specific page (128 bytes) from the buffer to the screen using DMA
void OLED_UpdatePage_DMA(uint8_t pageIdx, uint8_t* pBuffer);

// Check if DMA is currently busy
uint8_t OLED_IsDMABusy(void);

#endif /* __I2COLED_H */