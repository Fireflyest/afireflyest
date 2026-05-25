#include "extra_sensor.h"

uint8_t esRxBuffer[ES_RX_BUFFER_SIZE];
uint8_t esTxBuffer[ES_TX_BUFFER_SIZE];

__IO uint16_t esRxIndexUart2;
__IO uint8_t esRxStatusUart2;

void ExtraSensor_Init(uint32_t baudrate) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  // Enable USART2 clock

    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(USART2, DISABLE);
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);

    // Enable USART2 RX interrupt
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    // USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    // USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

uint16_t ExtraSensor_ReadData(uint8_t* data) {
    uint16_t size = esRxIndexUart2;
    if (size > 0) {
        memcpy(data, esRxBuffer, size);
        data[size] = '\0';  // Null-terminate the string
    }
    esRxStatusUart2 = ES_RX_STATE_IDLE;
    esRxIndexUart2 = 0;
    return size;
}

void ExtraSensor_WriteData(uint8_t* data, uint16_t size) {
    if (size == 0)
        return;

    
}