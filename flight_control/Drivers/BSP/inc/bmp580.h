#ifndef __BMP580_H
#define __BMP580_H

#include <stddef.h>
#include <stdint.h>
#include "bmp5_defs.h"
#include "spi_sensor.h"

/* ═══════════════════════════════════════════════════
 *  枚举
 * ═══════════════════════════════════════════════════ */

typedef enum {
    BMP580_MODE_STANDBY = 0x00,
    BMP580_MODE_NORMAL = 0x01,
    BMP580_MODE_FORCED = 0x02,
    BMP580_MODE_CONTINUOUS = 0x03,
} bmp580_mode_t;

typedef enum {
    BMP580_OSR_1X = 0x00,
    BMP580_OSR_2X = 0x01,
    BMP580_OSR_4X = 0x02,
    BMP580_OSR_8X = 0x03,
    BMP580_OSR_16X = 0x04,
    BMP580_OSR_32X = 0x05,
    BMP580_OSR_64X = 0x06,
    BMP580_OSR_128X = 0x07,
} bmp580_osr_t;

typedef enum {
    BMP580_ODR_0_125HZ = 0x00,
    BMP580_ODR_0_25HZ = 0x01,
    BMP580_ODR_0_5HZ = 0x02,
    BMP580_ODR_1HZ = 0x03,
    BMP580_ODR_2HZ = 0x04,
    BMP580_ODR_4HZ = 0x05,
    BMP580_ODR_8HZ = 0x06,
    BMP580_ODR_16HZ = 0x07,
    BMP580_ODR_32HZ = 0x08,
    BMP580_ODR_62HZ = 0x09,
    BMP580_ODR_125HZ = 0x0A,
    BMP580_ODR_250HZ = 0x0B,
    BMP580_ODR_500HZ = 0x0C,
} bmp580_odr_t;

typedef enum {
    BMP580_IIR_BYPASS = 0x00,
    BMP580_IIR_COEFF_1 = 0x01,
    BMP580_IIR_COEFF_3 = 0x02,
    BMP580_IIR_COEFF_7 = 0x03,
    BMP580_IIR_COEFF_15 = 0x04,
    BMP580_IIR_COEFF_31 = 0x05,
    BMP580_IIR_COEFF_63 = 0x06,
    BMP580_IIR_COEFF_127 = 0x07,
} bmp580_iir_t;

typedef enum {
    BMP580_FIFO_DISABLED = 0x00,
    BMP580_FIFO_TEMP_ONLY = 0x01,
    BMP580_FIFO_PRESS_ONLY = 0x02,
    BMP580_FIFO_PRESS_TEMP = 0x03,
} bmp580_fifo_frame_t;

typedef enum {
    BMP580_FIFO_STREAM = 0x00,
    BMP580_FIFO_STOP_FULL = 0x01,
} bmp580_fifo_mode_t;

typedef enum {
    BMP580_INT_SRC_DATA_RDY = (1u << 0),
    BMP580_INT_SRC_FIFO_FULL = (1u << 1),
    BMP580_INT_SRC_FIFO_THS = (1u << 2),
    BMP580_INT_SRC_OOR = (1u << 3),
} bmp580_int_src_t;

typedef enum {
    BMP580_OK = 0,
    BMP580_ERR_PARAM = -1,
    BMP580_ERR_CHIP_ID = -2,
    BMP580_ERR_COMM = -3,
    BMP580_ERR_NVM = -4,
    BMP580_ERR_TIMEOUT = -5,
    BMP580_ERR_FIFO_EMPTY = -6,
    BMP580_ERR_POWER_UP = -7,
    BMP580_ERR_POR = -8,
} bmp580_err_t;

/* ═══════════════════════════════════════════════════
 *  数据结构
 * ═══════════════════════════════════════════════════ */

typedef struct {
    int32_t raw_press;
    int32_t raw_temp;
    float pressure_pa;
    float temperature_deg;
} bmp580_data_t;

typedef struct {
    uint8_t sensor_id;
    uint8_t chip_id;
    bmp580_osr_t osr_p;
    bmp580_osr_t osr_t;
} bmp580_dev_t;

/* ═══════════════════════════════════════════════════
 *  延时
 * ═══════════════════════════════════════════════════ */

typedef void (*bmp580_delay_fn_t)(uint32_t ms);
void BMP580_SetDelay(bmp580_delay_fn_t fn);

/* ═══════════════════════════════════════════════════
 *  基础 API
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_WhoAmI(bmp580_dev_t* dev, uint8_t* chip_id);
bmp580_err_t BMP580_SoftReset(bmp580_dev_t* dev);
bmp580_err_t BMP580_Init(bmp580_dev_t* dev);
bmp580_err_t BMP580_SetPowerMode(bmp580_dev_t* dev, bmp580_mode_t mode);
bmp580_err_t BMP580_GetPowerMode(bmp580_dev_t* dev, bmp580_mode_t* mode);

/* ═══════════════════════════════════════════════════
 *  测量配置
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigOSR(bmp580_dev_t* dev,
                              bmp580_osr_t osr_p,
                              bmp580_osr_t osr_t);

bmp580_err_t BMP580_EnablePressure(bmp580_dev_t* dev, uint8_t enable);

bmp580_err_t BMP580_ConfigODR(bmp580_dev_t* dev, bmp580_odr_t odr);

bmp580_err_t BMP580_ConfigMeasurement(bmp580_dev_t* dev,
                                      bmp580_osr_t osr_p,
                                      bmp580_osr_t osr_t,
                                      bmp580_odr_t odr,
                                      uint8_t press_enable);

/* ═══════════════════════════════════════════════════
 *  IIR 滤波器
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigIIR(bmp580_dev_t* dev,
                              bmp580_iir_t iir_p,
                              bmp580_iir_t iir_t,
                              uint8_t flush_en);

bmp580_err_t BMP580_SetDataSrc(bmp580_dev_t* dev,
                               uint8_t src_t,
                               uint8_t src_p);

/* ═══════════════════════════════════════════════════
 *  数据读取
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ReadData(bmp580_dev_t* dev, bmp580_data_t* data);
bmp580_err_t BMP580_ForcedMeasure(bmp580_dev_t* dev, bmp580_data_t* data);

/* ═══════════════════════════════════════════════════
 *  FIFO
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigFIFO(bmp580_dev_t* dev,
                               bmp580_fifo_frame_t frame_sel,
                               bmp580_fifo_mode_t mode,
                               uint8_t dec_sel,
                               uint8_t threshold);

bmp580_err_t BMP580_GetFIFOCount(bmp580_dev_t* dev, uint8_t* count);
bmp580_err_t BMP580_ReadFIFO(bmp580_dev_t* dev,
                             uint8_t* buf,
                             uint32_t byte_count);

/* ═══════════════════════════════════════════════════
 *  中断
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_ConfigInt(bmp580_dev_t* dev,
                              uint8_t src_bitmap,
                              uint8_t active_high,
                              uint8_t open_drain,
                              uint8_t latched);

bmp580_err_t BMP580_GetIntStatus(bmp580_dev_t* dev, uint8_t* status);

/* ═══════════════════════════════════════════════════
 *  NVM
 * ═══════════════════════════════════════════════════ */

bmp580_err_t BMP580_NVMRead(bmp580_dev_t* dev,
                            uint8_t address,
                            uint16_t* data);

bmp580_err_t BMP580_NVMWrite(bmp580_dev_t* dev,
                             uint8_t address,
                             uint16_t data);

bmp580_err_t BMP580_ReadUID(bmp580_dev_t* dev, uint64_t* uid);

/* ═══════════════════════════════════════════════════
 *  高度计算
 * ═══════════════════════════════════════════════════ */

float BMP580_PressureToAltitude(float press_pa, float sea_level_pa);
float BMP580_PressureToAltitudeWithTemp(float press_pa,
                                        float temperature_c,
                                        float sea_level_pa);
float BMP580_AltitudeToSeaLevelPressure(float press_pa,
                                        float temperature_c,
                                        float altitude_m);

#endif /* __BMP580_H */
