# include "pwm.h"
# include <math.h>

uint16_t pwmDutyBuffer[4] = {0, 0, 0, 0};

void PWM_TIM_Init(uint16_t period, uint16_t prescaler) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = period - 1;           // 自动重载值
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;     // 预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;        // PWM 模式 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性高
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    
    TIM_Cmd(TIM3, ENABLE);
}


uint16_t PWM_Map_Percent(float percent) {
    percent = fminf(fmaxf((float)percent, 0.0f), 100.0f);
    return (uint16_t)(percent * (float)PWM_PERIOD / 100.0f);
}