#include "icm20948.h"


static uint8_t sensor_id = 0;

static void Delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 8000; j++);
}


/* Bank 切换 */
void ICM20948_Set_Bank(uint8_t bank) {
    SPI_Sensor_On(sensor_id);
    (void)SPI_Sensor_TransferByte(ICM20948_REG_BANK_SEL & 0x7F);  /* 写操作，去掉 0x80 */
    (void)SPI_Sensor_TransferByte((bank << 4) & 0x30);   /* Bank 值在 [5:4] 位 */
    SPI_Sensor_Off(sensor_id);
}

/* 读单寄存器 */
uint8_t ICM20948_Read_Reg(uint8_t reg) {
    uint8_t data;
    SPI_Sensor_On(sensor_id);
    (void)SPI_Sensor_TransferByte(reg | 0x80);
    data = SPI_Sensor_TransferByte(0x00);
    SPI_Sensor_Off(sensor_id);
    return data;
}

/* 写单寄存器 */
void ICM20948_Write_Reg(uint8_t reg, uint8_t data) {
    SPI_Sensor_On(sensor_id);
    (void)SPI_Sensor_TransferByte(reg & 0x7F);
    (void)SPI_Sensor_TransferByte(data);
    SPI_Sensor_Off(sensor_id);
}

/* 连续读寄存器 */
void ICM20948_Read_Regs(uint8_t reg, uint8_t *buf, uint16_t len) {
    SPI_Sensor_On(sensor_id);
    (void)SPI_Sensor_TransferByte(reg | 0x80);
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = SPI_Sensor_TransferByte(0x00);
    }
    SPI_Sensor_Off(sensor_id);
}

/* 单独读取 WHO_AM_I 用于调试 */
uint8_t ICM20948_Read_WhoAmI(void) {
    ICM20948_Set_Bank(0);  /* 确保在 Bank 0 */
    return ICM20948_Read_Reg(ICM20948_REG_WHO_AM_I);
}

/* 读取磁力计 WHO_AM_I 用于调试 */
uint8_t ICM20948_Read_MagWhoAmI(void) {
    uint8_t mag_id = 0;
    uint8_t addr_test[2] = {0x0C, 0x0D};
    
    ICM20948_Set_Bank(0);
    
    for (int i = 0; i < 2; i++) {
        /* 临时配置 SLV0 读取磁力计 ID 寄存器 */
        ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_ADDR, 0x80 | addr_test[i]);
        ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_REG, AK09916_REG_WIA2);
        ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_CTRL, 0x80 | 0x01);
        
        Delay_ms(30);
        
        mag_id = ICM20948_Read_Reg(ICM20948_REG_EXT_SENS_DATA_00);
        
        if (mag_id == 0x09) {
            break;
        }
    }
    
    return mag_id;
}

/* 设备初始化 */
void ICM20948_Init(void) {
    sensor_id = SPI_Sensor_Register(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);

    uint8_t who_am_i;
    uint8_t buffer[10];

    /* 1. 复位 */
    ICM20948_Set_Bank(0);
    ICM20948_Write_Reg(ICM20948_REG_PWR_MGMT_1, BIT_RESET);
    Delay_ms(100);

    /* 2. 检查 ID */
    who_am_i = ICM20948_Read_Reg(ICM20948_REG_WHO_AM_I);
    // if (who_am_i != 0xEA) {
    //     return;
    // }

    /* 3. 唤醒 (时钟源) */
    ICM20948_Write_Reg(ICM20948_REG_PWR_MGMT_1, BIT_CLK_PLL);
    Delay_ms(10);

    /* 4. 禁用所有传感器功耗管理 */
    ICM20948_Set_Bank(0);
    ICM20948_Write_Reg(ICM20948_REG_PWR_MGMT_2, 0x00);

    /* 5. 配置 USER_CTRL - 先复位 I2C Master */
    ICM20948_Write_Reg(ICM20948_REG_USER_CTRL, BIT_I2C_MST_RST | BIT_I2C_IF_DIS);
    Delay_ms(10);
    
    /* 6. 启用 I2C Master */
    ICM20948_Write_Reg(ICM20948_REG_USER_CTRL, BIT_I2C_MST_EN | BIT_I2C_IF_DIS);
    Delay_ms(10);

    /* 7. 配置 Bank 3 */
    ICM20948_Set_Bank(3);
    ICM20948_Write_Reg(ICM20948_REG_I2C_MST_ODR_CONFIG, 0x01);
    ICM20948_Set_Bank(0);
    Delay_ms(10);

    /* 8. 配置 I2C_MST_CTRL */
    ICM20948_Write_Reg(ICM20948_REG_I2C_MST_CTRL, 0x07);  /* 360kHz */
    Delay_ms(10);

    /* 9. SLV0 写 - 磁力计软复位 */
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR);
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_REG, AK09916_REG_CNTL3);
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_DO, AK09916_MODE_RESET);
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_CTRL, 0x80 | 0x01);
    Delay_ms(100);

    /* 10. SLV0 写 - 磁力计连续模式 */
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR);
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_REG, AK09916_REG_CNTL2);
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_DO, AK09916_MODE_CONT);
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV0_CTRL, 0x80 | 0x01);
    Delay_ms(150);

    /* 11. 【关键】SLV1 读 - 从 ST1 开始读 8 字节 */
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV1_ADDR, 0x80 | AK09916_I2C_ADDR);
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV1_REG, AK09916_REG_ST1);
    ICM20948_Write_Reg(ICM20948_REG_I2C_SLV1_CTRL, 0x80 | 0x08);
    Delay_ms(100);

    /* 12. 传感器量程 */
    ICM20948_Set_Bank(2);
    // ±2000dps: GYRO_FS_SEL=3 (bits [2:1] = 11)
    ICM20948_Write_Reg(ICM20948_REG_GYRO_CONFIG_1, (3 << 1) | 0x01);
    // ±16g: ACCEL_FS_SEL=3 (bits [2:1] = 11)
    ICM20948_Write_Reg(ICM20948_REG_ACCEL_CONFIG, (3 << 1) | 0x01);
    ICM20948_Write_Reg(ICM20948_REG_ACCEL_CONFIG_2, 0x03);
    ICM20948_Set_Bank(0);
}

/* 读取所有传感器数据 */
void ICM20948_Read(uint8_t *imu_rx_buf, uint8_t *mag_rx_buf) {
    ICM20948_Set_Bank(0);

    /* 读取 IMU 数据 (Accel 6 + Gyro 6 + Temp 2 = 14 字节) */
    ICM20948_Read_Regs(ICM20948_REG_ACCEL_OUT_H, imu_rx_buf, 14);

    /* 读取磁力计数据 (6 字节 XYZ 和状态，从 EXT_SENS_DATA_00 开始) */
    ICM20948_Read_Regs(ICM20948_REG_EXT_SENS_DATA_00, mag_rx_buf, 6);
}
