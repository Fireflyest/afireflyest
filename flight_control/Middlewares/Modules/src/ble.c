#include "ble.h"

uint8_t bleRxBuffer[BLE_RX_BUFFER_SIZE];
uint8_t bleTxBuffer[BLE_TX_BUFFER_SIZE];

__IO uint16_t bleRxIndexUart1;
__IO uint8_t bleRxStatusUart1;

void BLE_Init(uint32_t baudrate) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Enable USART1 clock

    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(USART1, DISABLE);
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);

    // Enable USART1 RX interrupt
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}

uint16_t BLE_ReadData(uint8_t* data) {
    uint16_t size = bleRxIndexUart1;
    if (size > 0) {
        memcpy(data, bleRxBuffer, size);
        data[size] = '\0'; // Null-terminate the string
    }
    bleRxStatusUart1 = BLE_RX_STATE_IDLE;
    bleRxIndexUart1 = 0;
    return size;
}

void BLE_WriteData(uint8_t* data, uint16_t size) {
    if (size == 0) return;

    uint16_t timeout;
    for (timeout = 0xFFFF;DMA_GetCmdStatus(DMA2_Stream7) != DISABLE && timeout > 0; timeout--) ;
    if (timeout == 0) return;

    uint16_t sendLen = (size > BLE_TX_BUFFER_SIZE) ? BLE_TX_BUFFER_SIZE : size;

    memcpy(bleTxBuffer, data, sendLen);

    DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);
    DMA_SetCurrDataCounter(DMA2_Stream7, sendLen);
    DMA_Cmd(DMA2_Stream7, ENABLE);
}