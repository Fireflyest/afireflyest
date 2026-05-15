# ifndef __PWM_H
# define __PWM_H

# include "stm32f4xx.h"


// 10kHz: 84MHz / 84 / 100 = 10kHz  → period=100, 精度太差
// 10kHz: 84MHz / 1 / 8400 = 10kHz  → period=8400, 精度好
// 8kHz:  84MHz / 1 / 10500 = 8kHz  → period=10500
// 20kHz: 84MHz / 1 / 4200 = 20kHz  → period=4200

#define PWM_TIM_CLOCK_HZ    84000000UL

#define PWM_FREQ_HZ         10000

#define PWM_PERIOD     (PWM_TIM_CLOCK_HZ / PWM_FREQ_HZ)       // 84MHz / 1 / 8400 = 10kHz
#define PWM_PRESCALER  1          // 无分频

#define PWM_MAX_DUTY   PWM_PERIOD
#define PWM_MIN_DUTY   0


extern uint16_t pwmDutyBuffer[4];

void PWM_TIM_Init(uint16_t period, uint16_t prescaler);
uint16_t PWM_Map_Percent(float percent);

# endif /* __PWM_H */