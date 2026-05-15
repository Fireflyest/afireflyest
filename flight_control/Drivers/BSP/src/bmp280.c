#include "bmp280.h"


typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp_calib_t;

bmp_calib_t bmp_calib;
uint8_t bmp_rx_buf[6];

static float temperature;
static float barometricPressure;
static float altitude;
static int t_fine;

static uint8_t sensor_id = 0;

static void BMP280_Register_Write(uint8_t reg, uint8_t data) {
    // cs on
    SPI_Sensor_On(sensor_id);
    // send register address (ensure MSB = 0 for write)
    (void)SPI_Sensor_TransferByte((uint8_t)(reg & 0x7F));
    // send data
    (void)SPI_Sensor_TransferByte(data);
    // cs off
    SPI_Sensor_Off(sensor_id);
}

static void BMP280_Calibration_Read(void) {
    uint8_t b[24];
    
    // cs on
    SPI_Sensor_On(sensor_id);
    
    // send start register address with Read bit (0x88 | 0x80 = 0x88 starting address)
    (void)SPI_Sensor_TransferByte(0x88 | 0x80);
    
    // continuous read 24 bytes
    for (int i = 0; i < 24; ++i) {
        b[i] = SPI_Sensor_TransferByte(0xFF);
    }
    
    // cs off
    SPI_Sensor_Off(sensor_id);

    bmp_calib.dig_T1 = (uint16_t)(b[1] << 8 | b[0]);
    bmp_calib.dig_T2 = (int16_t)(b[3] << 8 | b[2]);
    bmp_calib.dig_T3 = (int16_t)(b[5] << 8 | b[4]);
    bmp_calib.dig_P1 = (uint16_t)(b[7] << 8 | b[6]);
    bmp_calib.dig_P2 = (int16_t)(b[9] << 8 | b[8]);
    bmp_calib.dig_P3 = (int16_t)(b[11] << 8 | b[10]);
    bmp_calib.dig_P4 = (int16_t)(b[13] << 8 | b[12]);
    bmp_calib.dig_P5 = (int16_t)(b[15] << 8 | b[14]);
    bmp_calib.dig_P6 = (int16_t)(b[17] << 8 | b[16]);
    bmp_calib.dig_P7 = (int16_t)(b[19] << 8 | b[18]);
    bmp_calib.dig_P8 = (int16_t)(b[21] << 8 | b[20]);
    bmp_calib.dig_P9 = (int16_t)(b[23] << 8 | b[22]);
}


static void BMP280_Compensate_T(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp_calib.dig_T1 << 1))) * ((int32_t)bmp_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1))) >> 12) * ((int32_t)bmp_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;
}

static void BMP280_Compensate_P(int32_t adc_P) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)bmp_calib.dig_P1) >> 33;
    if (var1 == 0) return; // avoid div0
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp_calib.dig_P7) << 4);
    barometricPressure = p / 25600.0f + BMP_BAROMETERIC_PRESSURE_OFFSET; // convert fixed-point to float Pa
}

static void BMP280_GetAltitude() {
    float T = temperature + 273.15f; 
    // 0.190263 是 R*L/g 的常数
    const float EXP = 0.190263f; 
    
    // 使用实时温度 T 替代固定常数 44330 (44330 实际上包含了 288.15K 的假设)
    // 高度 = (T / 0.0065) * (1 - (P/P0)^EXP)
    altitude = (T / 0.0065f) * (1.0f - powf(barometricPressure / BMP_SEA_LEVEL_PRESSURE_HPA, EXP));
}

uint8_t BMP280_Register_Read(uint8_t reg) {
    uint8_t v;
    // cs on
    SPI_Sensor_On(sensor_id);
    for (volatile int i = 0; i < 200; ++i); // short settle delay
    (void)SPI_Sensor_TransferByte((uint8_t)(reg | 0x80)); // read flag
    v = SPI_Sensor_TransferByte(0xFF);
    // cs off
    SPI_Sensor_Off(sensor_id);
    return v;
}

void BMP280_Init(void) {
    sensor_id = SPI_Sensor_Register(GPIO_BMP_SPI, GPIO_BMP_SPI_CS_PIN);

    BMP280_Register_Write(BMP_CONFIG, 0xA0);    // standby 1000ms, filter off
    BMP280_Register_Write(BMP_CTRL_MEAS, 0x27); // normal mode, temp and pressure oversampling x1
    BMP280_Calibration_Read();
}

uint8_t BMP280_Read_WhoAmI(void) {
    return BMP280_Register_Read(0xD0); // WHO_AM_I register
}

void BMP280_Read(float* alt, float *tmp) {
    // cs on
    SPI_Sensor_On(sensor_id);
    // send register address with Read bit (typically MSB=1 for BMP SPI read)
    (void)SPI_Sensor_TransferByte((uint8_t)(0xF7 | 0x80)); // Example register for BMP data
    for (int i = 0; i < 6; ++i) { // assuming 6 bytes of data
        bmp_rx_buf[i] = SPI_Sensor_TransferByte(0xFF);
    }
    // cs off
    SPI_Sensor_Off(sensor_id);

    int32_t adc_P = ((int32_t)bmp_rx_buf[0] << 12)
                   | ((int32_t)bmp_rx_buf[1] << 4)
                   | ((int32_t)bmp_rx_buf[2] >> 4);

    // 温度: 20bit, msb[7:0] lsb[7:0] xlsb[7:4]
    int32_t adc_T = ((int32_t)bmp_rx_buf[3] << 12)
                   | ((int32_t)bmp_rx_buf[4] << 4)
                   | ((int32_t)bmp_rx_buf[5] >> 4);

    BMP280_Compensate_T(adc_T);   // → temperature, t_fine
    BMP280_Compensate_P(adc_P);   // → barometricPressure
    BMP280_GetAltitude();
    *alt = altitude;
    *tmp = temperature;
}

