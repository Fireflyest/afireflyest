#include "accel_gyro.h"
#include <math.h>
#include <string.h>
#include "SPI_Sensor.h"
#include "bmi260.h"
#include "bmi2_defs.h"

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH_MS2 (9.80665f)

/*! Macros to select the sensors                   */
#define ACCEL UINT8_C(0x00)
#define GYRO UINT8_C(0x01)

/* ═══════════════════════════════════════════════
 *  内部状态
 * ═══════════════════════════════════════════════ */

static struct bmi2_dev g_bmi;
static uint8_t g_spi_id;

/* ═══════════════════════════════════════════════
 *  微秒延时（简单循环，不含 DWT/SysTick）
 *  SystemCoreClock=168MHz 时约 42 次/μs
 *  实际偏长，但 Bosch 驱动对延时容错很大
 * ═══════════════════════════════════════════════ */

static void bmi2_delay_us(uint32_t period, void* intf_ptr) {
    (void)intf_ptr;
    /* 每次循环约 3~4 个时钟周期 */
    volatile uint32_t count = period * (SystemCoreClock / 1000000 / 4);
    while (count--)
        ;
}

/* ═══════════════════════════════════════════════
 *  Bosch 驱动 SPI 回调
 *
 *  Bosch 驱动内部已经处理了:
 *    - 读: reg_addr |= 0x80  →  调用 read()
 *    - 写: reg_addr &= 0x7F  →  调用 write()
 *    - dummy_byte: read() 多读 1 字节，再跳过
 *  所以回调直接透传，不做任何额外处理。
 * ═══════════════════════════════════════════════ */

static BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr,
                                           uint8_t* reg_data,
                                           uint32_t len,
                                           void* intf_ptr) {
    uint8_t id = *(uint8_t*)intf_ptr;
    return (BMI2_INTF_RETURN_TYPE)SPI_Sensor_ReadBytes(id, reg_addr,
                                                       reg_data, (uint16_t)len);
}

static BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr,
                                            const uint8_t* reg_data,
                                            uint32_t len,
                                            void* intf_ptr) {
    uint8_t id = *(uint8_t*)intf_ptr;
    return (BMI2_INTF_RETURN_TYPE)SPI_Sensor_WriteBytes(id, reg_addr,
                                                        reg_data, (uint16_t)len);
}

/* ═══════════════════════════════════════════════
 *  BMI_Init — 完全参照官方 example 流程
 *
 *  官方流程:
 *    bmi2_interface_init()  → 填充 bmi2_dev 结构
 *    bmi260_init()          → 读 chip_id + 软复位 + 上传配置文件
 *    set_accel_gyro_config()→ 配置 ACC/GYR 参数
 *    bmi2_sensor_enable()   → 使能传感器
 * ═══════════════════════════════════════════════ */

static int8_t set_accel_gyro_config(struct bmi2_dev* bmi);

int BMI_Init(uint8_t spi_id) {
    int8_t rslt;
    uint8_t dummy;

    g_spi_id = spi_id;

    /*
     * ── bmi2_interface_init 对应部分 ──
     * 参照官方 common.c 中 bmi2_interface_init 的 SPI 分支
     */
    memset(&g_bmi, 0, sizeof(g_bmi));

    g_bmi.intf = BMI2_SPI_INTF;
    g_bmi.read = bmi2_spi_read;
    g_bmi.write = bmi2_spi_write;
    g_bmi.delay_us = bmi2_delay_us;
    g_bmi.intf_ptr = &g_spi_id;
    g_bmi.read_write_len = 256;
    g_bmi.config_file_ptr = NULL;
    /* 注意: dummy_byte 由 bmi260_init 内部设置，不要在这里赋值 */

    /*
     * BMI260 上电默认 I2C 模式。
     * 数据手册 3.2.1: 一次读 0x00 操作触发 I2C→SPI 切换。
     * 这次 dummy read 必须在 bmi260_init 之前完成，
     * 否则 bmi260_init 内部读 chip_id 时芯片还没切到 SPI 模式。
     */
    SPI_Sensor_ReadBytes(spi_id, 0x00, &dummy, 1);
    bmi2_delay_us(250000, NULL); /* 50ms 确保切换完成 */

    /*
     * ── bmi260_init ──
     * 内部流程: 设置 dummy_byte → 读 chip_id → 软复位
     *         → 上传配置文件 → 等待初始化完成
     */
    bmi2_delay_us(2500000, NULL);
    // rslt = bmi270_init(&g_bmi);
    for (int i = 0; i < 50; i++) {
        rslt = bmi270_init(&g_bmi);
        if (rslt != BMI2_E_DEV_NOT_FOUND) {
            break;
        }
        bmi2_delay_us(50000, NULL);
    }
    if (rslt != BMI2_OK) {
        return (int)rslt;
    }

    /*
     * ── set_accel_gyro_config ──
     * 完全参照官方 accel_gyro example
     */
    rslt = set_accel_gyro_config(&g_bmi);
    if (rslt != BMI2_OK) {
        return (int)rslt;
    }

    /*
     * ── bmi2_sensor_enable ──
     * 官方注释: Accel and Gyro enable must be done after setting configurations
     */
    uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    rslt = bmi2_sensor_enable(sensor_list, 2, &g_bmi);
    if (rslt != BMI2_OK) {
        return (int)rslt;
    }

    return 0;
}

/*
 * 照搬官方 set_accel_gyro_config，去掉了 interrupt mapping
 */
static int8_t set_accel_gyro_config(struct bmi2_dev* bmi) {
    int8_t rslt;
    struct bmi2_sens_config config[2];

    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    if (rslt != BMI2_OK)
        return rslt;

    /* Accel: 200Hz, ±4g, normal avg 4, 高性能 */
    config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
    config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_4G;
    config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    /* Gyro: 200Hz, ±2000dps, normal bandwidth, 高性能 */
    config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
    config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
    config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    /* Set the accel and gyro configurations */
    rslt = bmi2_set_sensor_config(config, 2, bmi);

    return rslt;
}

/* ═══════════════════════════════════════════════
 *  WhoAmI（SPI dummy byte 处理）
 * ═══════════════════════════════════════════════ */

int BMI_WhoAmI(uint8_t* id) {
    if (id == NULL)
        return -1;

    /* SPI 读 chip_id 时第一个字节是 dummy，第二个才是真实数据 */
    uint8_t buf[2];
    int ret = SPI_Sensor_ReadBytes(g_spi_id, 0x00, buf, 2);
    if (ret != 0)
        return -1;

    *id = buf[1];
    return 0;
}

/* ═══════════════════════════════════════════════
 *  数据读取（参照官方 bmi2_get_sensor_data）
 * ═══════════════════════════════════════════════ */

int BMI_ReadAccelGyro(bmi_raw_data_t* acc, bmi_raw_data_t* gyr) {
    struct bmi2_sens_data sd;
    int8_t rslt;

    sd.status = 0;
    rslt = bmi2_get_sensor_data(&sd, &g_bmi);
    if (rslt != BMI2_OK)
        return (int)rslt;

    if (acc && (sd.status & BMI2_DRDY_ACC)) {
        acc->x = sd.acc.x;
        acc->y = sd.acc.y;
        acc->z = sd.acc.z;
    }

    if (gyr && (sd.status & BMI2_DRDY_GYR)) {
        gyr->x = sd.gyr.x;
        gyr->y = sd.gyr.y;
        gyr->z = sd.gyr.z;
    }

    return 0;
}

int BMI_ReadAccel(bmi_raw_data_t* data) {
    return BMI_ReadAccelGyro(data, NULL);
}

int BMI_ReadGyro(bmi_raw_data_t* data) {
    return BMI_ReadAccelGyro(NULL, data);
}

/* ═══════════════════════════════════════════════
 *  物理量转换（与官方 lsb_to_mps2 / lsb_to_dps 一致）
 * ═══════════════════════════════════════════════ */

void BMI_AccelToMps2(const bmi_raw_data_t* raw, float g_range, uint8_t bit_width, float out[3]) {
    float half_scale = (float)((1UL << bit_width) / 2.0f);

    out[0] = (GRAVITY_EARTH_MS2 * raw->x * g_range) / half_scale;
    out[1] = (GRAVITY_EARTH_MS2 * raw->y * g_range) / half_scale;
    out[2] = (GRAVITY_EARTH_MS2 * raw->z * g_range) / half_scale;
}

void BMI_GyroToRads(const bmi_raw_data_t* raw, float dps, uint8_t bit_width, float out[3]) {
    float half_scale = (float)((1UL << bit_width) / 2.0f);

    /* dps → rad/s 需要乘以 π/180 */
    out[0] = (dps * raw->x) / half_scale * 0.01745329251994f;
    out[1] = (dps * raw->y) / half_scale * 0.01745329251994f;
    out[2] = (dps * raw->z) / half_scale * 0.01745329251994f;
}

/* ═══════════════════════════════════════════════
 *  辅助函数
 * ═══════════════════════════════════════════════ */

struct bmi2_dev* BMI_GetDev(void) {
    return &g_bmi;
}

void BMI_PrintError(int8_t rslt) {
    switch (rslt) {
    case BMI2_OK:
        break;
    case BMI2_E_NULL_PTR:
        printf("[BMI] null ptr\n");
        break;
    case BMI2_E_COM_FAIL:
        printf("[BMI] comm fail\n");
        break;
    case BMI2_E_DEV_NOT_FOUND:
        printf("[BMI] chip ID mismatch\n");
        break;
    case BMI2_E_CONFIG_LOAD:
        printf("[BMI] config load fail\n");
        break;
    case BMI2_E_INVALID_SENSOR:
        printf("[BMI] invalid sensor\n");
        break;
    default:
        printf("[BMI] err %d\n", rslt);
        break;
    }
}