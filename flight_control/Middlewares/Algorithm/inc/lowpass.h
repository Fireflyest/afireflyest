#ifndef __LOWPASS_H
#define __LOWPASS_H

typedef struct {
    float alpha;    // 滤波系数，范围 [0.0, 1.0]
    float output;   // 上次滤波输出值
} LowPass_Filter_t;

void LowPass_Filter_Init(LowPass_Filter_t* filter, float alpha, float initialOutput);

void LowPass_Update(LowPass_Filter_t* filter, float input);

/*
* 使用时间常数 tau 和时间步 dt 更新低通滤波器
* tau: 时间常数，单位秒
* dt: 时间步，单位秒
*/
void LowPass_UpdateWithTau(LowPass_Filter_t* filter, float input, float tau, float dt);

#endif /* __LOWPASS_H */