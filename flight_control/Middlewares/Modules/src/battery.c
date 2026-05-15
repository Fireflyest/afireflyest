# include "battery.h"

__IO uint16_t pwr_adc_raw = 0;
__IO uint16_t pwr_state = PWR_STATE_PREPARE;

static uint8_t rc_counter = 0;
static uint8_t disable_counter = 0;

static const uint16_t li_po_curve[11] = {
    4200, // 100%
    4060, // 90%
    3980, // 80%
    3920, // 70%
    3870, // 60%
    3820, // 50%
    3790, // 40%
    3770, // 30%
    3740, // 20%
    3680, // 10%
    3300  // 0%
};


void Battery_ADC_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
}

void Battery_Measure_Reset(void) {
    pwr_state = PWR_STATE_PREPARE;
}

void Battery_Measure_Step(void) {
    if (pwr_state == PWR_STATE_PREPARE) {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_PWR_AUX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIO_PWR_AUX, &GPIO_InitStructure);
        GPIO_SetBits(GPIO_PWR_AUX, GPIO_PWR_AUX_PIN);
        pwr_state = PWR_STATE_WAIT_RC;
    } else if (pwr_state == PWR_STATE_WAIT_RC) {
        rc_counter++;
        if (rc_counter >= 8) {
            rc_counter = 0;
            pwr_state = PWR_STATE_READ;
        }
    } else if (pwr_state == PWR_STATE_READ) {
        ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles); // PC0 is ADC Channel 10
        ADC_SoftwareStartConv(ADC1);
        pwr_state = PWR_STATE_WAIT_ADC;
    } else if (pwr_state == PWR_STATE_WAIT_ADC) {
        // 等待ADC转换完成，结果将在中断服务程序中处理
    } else if (pwr_state == PWR_STATE_DISABLE) {
        GPIO_ResetBits(GPIO_PWR_AUX, GPIO_PWR_AUX_PIN);
        disable_counter++;
        if (disable_counter >= 1000) {
            disable_counter = 0;
            pwr_state = PWR_STATE_PREPARE;
        }
    }
}

uint8_t Battery_GetPercentage(void) {
    static uint32_t filtered_mv = 0;
    
    // 1. 使用宏定义的常量进行纯整数运算 (mV)
    // 公式解析: (ADC值 * 参考电压mV * 分压比) / ADC最大值
    // 为了防止溢出并保持精度，先乘后除
    uint32_t current_mv = (uint32_t)((float)pwr_adc_raw * ADC_RAW_TO_MV_FACTOR);

    // 2. 指数移动平均滤波 (EMA)
    if (filtered_mv == 0) {
        filtered_mv = current_mv;
    } else {
        filtered_mv = (filtered_mv * 9 + current_mv) / 10;
    }

    // 3. 边界限制 (使用宏定义)
    if (filtered_mv >= BATTERY_FULL) return 100;
    if (filtered_mv <= BATTERY_EMPTY) return 0;

    // 4. 查表计算百分比 (基于 li_po_curve)
    for (int i = 0; i < 10; i++) {
        if (filtered_mv >= li_po_curve[i+1]) {
            uint32_t voltage_diff = li_po_curve[i] - li_po_curve[i+1];
            uint32_t voltage_above_base = filtered_mv - li_po_curve[i+1];
            
            uint8_t base_pct = (9 - i) * 10;
            uint8_t extra_pct = (voltage_above_base * 10) / voltage_diff;
            
            return base_pct + extra_pct;
        }
    }
    return 0;
}

uint32_t Battery_GetVoltage(void) {
    uint32_t current_mv = (uint32_t)((float)pwr_adc_raw * ADC_RAW_TO_MV_FACTOR);
    return current_mv;
}
