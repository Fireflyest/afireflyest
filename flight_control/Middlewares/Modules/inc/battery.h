#ifndef __BATTERY_H
#define __BATTERY_H

#include "board.h"
#include "stm32f4xx.h"

/* ══════════════════════════════════════════════════════════════
 *  分压器参数 (R1 = R2 = 100kΩ)
 *
 *  Vbat ──[R1]──┬── ADC
 *               │
 *             [R2]
 *               │
 *           MOSFET ── GND
 *
 *  Vadc = Vbat × R2/(R1+R2) = Vbat × 0.5
 *  Vbat = Vadc × 2
 * ══════════════════════════════════════════════════════════════ */

#define R_TOP_OHM 100000.0f
#define R_BOTTOM_OHM 100000.0f

#define VREF_V 3.3f
#define BATTERY_ADC_MAX 4095.0f
#define VOLTAGE_DIVIDER_RATIO 2.0f /* Vbat = Vadc × 2 (R1=R2=100kΩ) */
#define ADC_RAW_TO_MV_FACTOR ((VREF_V * 1000.0f / VOLTAGE_DIVIDER_RATIO) / BATTERY_ADC_MAX)
/* 展开: (3300 / 2) / 4095 = 0.4029 mV/LSB
 * 4095 × 0.4029 = 1649 mV (ADC端) × 2 = 3299 mV (电池端) ≈ 3.3V ✓ */

#define BATTERY_FULL 4200  /* 4.2V */
#define BATTERY_EMPTY 3300 /* 3.3V */

/* 状态机 */
#define PWR_STATE_PREPARE 0 /* 开启 MOSFET，等待 RC 稳定 */
#define PWR_STATE_WAIT_RC 1 /* 开启 MOSFET，等待 RC 稳定 */
#define PWR_STATE_READ 2    /* 启动 ADC 转换 */
#define PWR_STATE_WAIT_ADC 3    /* 等待 ADC 转换完成 */
#define PWR_STATE_DISABLE 4 /* 关闭 MOSFET，保存结果 */

extern __IO uint16_t pwr_adc_raw;
extern __IO uint16_t pwr_state;

void Battery_ADC_Init(void);
void Battery_Measure_Reset(void);
void Battery_Measure_Step(void);
uint8_t Battery_GetPercentage(void);
uint32_t Battery_GetVoltage(void);

#endif /* __BATTERY_H */
