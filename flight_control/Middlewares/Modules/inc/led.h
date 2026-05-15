#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
#include "board.h"

#define LED_TOGGLE_INTERVAL            3

#define LED_TOGGLE_CMD_ALL_ON          0b11111111
#define LED_TOGGLE_CMD_ALL_OFF         0b00000000
#define LED_TOGGLE_CMD_BLINK_FAST      0b10101010
#define LED_TOGGLE_CMD_BLINK_NORMAL    0b11001100
#define LED_TOGGLE_CMD_BLINK_SLOW      0b11110000


void LED_On(void);
void LED_Off(void);
void LED_Blink(uint8_t toggleCmd, uint8_t times);

void LED_Toggle_Handler(void);

# endif /* __LED_H */