#ifndef __FPS_H
#define __FPS_H

#include "stm32f4xx.h"

extern __IO uint32_t systemTick;

void Delay_ms(__IO uint32_t nTime);

void FPS_StartFrame(void);
void FPS_EndFrame(void);

uint32_t FPS_Get(void);
float FPS_GetDeltaTime(void);

#endif /* __FPS_H */