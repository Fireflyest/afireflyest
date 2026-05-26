#ifndef __BMI260_H
#define __BMI260_H

#include <stddef.h>
#include <stdint.h>
#include "bmi2_defs.h"
#include "spi_sensor.h"

/* ═══════════════════════════════════════════════════
 *  BMI260 专用常量
 * ═══════════════════════════════════════════════════ */

#define BMI260_CHIP_ID 0x27

extern const uint8_t bmi260_config_file[];
extern const uint32_t bmi260_config_size;

/* ═══════════════════════════════════════════════════
 *  错误码
 * ═══════════════════════════════════════════════════ */

typedef enum {
    BMI260_OK = 0,
    BMI260_ERR_PARAM = -1,       /* 参数无效（NULL指针） */
    BMI260_ERR_CHIP_ID = -2,     /* Chip ID 校验失败 */
    BMI260_ERR_RESET = -3,       /* 软复位写入失败 */
    BMI260_ERR_WAKEUP = -4,      /* 复位后唤醒超时 */
    BMI260_ERR_CONFIG = -5,      /* 上传配置文件失败 */
    BMI260_ERR_APS_DISABLE = -6, /* 关闭 APS 模式失败 */
    BMI260_ERR_ENABLE = -7,      /* 使能传感器失败 */
    BMI260_ERR_ACC_RANGE = -8,   /* 设置 ACC 量程失败 */
    BMI260_ERR_GYR_RANGE = -9,   /* 设置 GYR 量程失败 */
} bmi260_err_t;

/* ═══════════════════════════════════════════════════
 *  数据结构
 * ═══════════════════════════════════════════════════ */

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} bmi260_raw_xyz_t;

typedef struct {
    bmi260_raw_xyz_t acc;
    bmi260_raw_xyz_t gyr;
    int16_t temp_raw;
} bmi260_data_t;

typedef struct {
    uint8_t sensor_id;
    uint8_t aps_status;
    uint8_t acc_range;
    uint8_t gyr_range;
    float acc_lsb_per_g;
    float gyr_lsb_per_dps;
} bmi260_dev_t;

/* ═══════════════════════════════════════════════════
 *  延时
 * ═══════════════════════════════════════════════════ */

typedef void (*bmi260_delay_fn_t)(uint32_t ms);
void BMI260_SetDelay(bmi260_delay_fn_t fn);

/* ═══════════════════════════════════════════════════
 *  API
 * ═══════════════════════════════════════════════════ */

bmi260_err_t BMI260_WhoAmI(bmi260_dev_t* dev, uint8_t* chip_id);
bmi260_err_t BMI260_Init(bmi260_dev_t* dev);
bmi260_err_t BMI260_ReadAll(bmi260_dev_t* dev, bmi260_data_t* data);
bmi260_err_t BMI260_ReadAccel(bmi260_dev_t* dev, bmi260_raw_xyz_t* acc);
bmi260_err_t BMI260_ReadGyro(bmi260_dev_t* dev, bmi260_raw_xyz_t* gyr);
float BMI260_RawToTemp(int16_t raw);

#endif /* __BMI260_H */
