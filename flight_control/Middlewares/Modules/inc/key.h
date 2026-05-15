#ifndef __KEY_H
#define __KEY_H	 

#include "stm32f4xx_conf.h"
#include "board.h"

#define KEY_STATE_RELEASED    0
#define KEY_STATE_PRESSED     1

#define KEY_DEBOUNCE_TIME     1

uint8_t Key_Status();
uint8_t Key_PressConsume();
void Key_Callback(void (*onPress)(void), void (*onRelease)(void));
void Key_Toggle_Handler(void);

#endif

