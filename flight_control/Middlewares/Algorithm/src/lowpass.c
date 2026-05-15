# include "lowpass.h"
# include <math.h>

void LowPass_Filter_Init(LowPass_Filter_t* filter, float alpha, float initialOutput) {
    alpha = fmaxf(0.0f, fminf(1.0f, alpha)); // 限制 alpha 在 [0.0, 1.0]
    filter->alpha = alpha;
    filter->output = initialOutput;
}

void LowPass_Update(LowPass_Filter_t* filter, float input) {
    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->output;
}

void LowPass_UpdateWithTau(LowPass_Filter_t* filter, float input, float tau, float dt) {
    float alpha = dt / (tau + dt);
    alpha = fmaxf(0.0f, fminf(1.0f, alpha)); // 限制 alpha 在 [0.0, 1.0]
    filter->output = alpha * input + (1.0f - alpha) * filter->output;
}
