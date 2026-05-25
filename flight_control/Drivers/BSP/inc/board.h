#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f4xx.h"

#define GPIO_LED GPIOC
#define GPIO_LED_PIN GPIO_Pin_13

#define GPIO_KEY GPIOA
#define GPIO_KEY_PIN GPIO_Pin_0

#define SPI_SENSOR SPI2
#define GPIO_IMU_SPI GPIOB
#define GPIO_IMU_SPI_CS_PIN GPIO_Pin_12
#define SENSOR_BARO
#define GPIO_BARO_SPI GPIOC
#define GPIO_BARO_SPI_CS_PIN GPIO_Pin_2
#define SENSOR_MAG
#define GPIO_MAG_SPI GPIOD
#define GPIO_MAG_SPI_CS_PIN GPIO_Pin_2

#define GPIO_PWR_AUX GPIOC
#define GPIO_PWR_AUX_PIN GPIO_Pin_1

extern uint8_t spi2Occupied; // 0: idle, 1: ICM-20948, 2: BMP280

/**
 * @brief Initialize the GPIO pins for the LED
 * LED connected to PC13, active low
 */
void LED_GPIO_Init(void);

/**
 * @brief Initialize the GPIO pins for the Key
 * Key connected to PA0, active low
 */
void Key_GPIO_Init(void);

/**
 * @brief Initialize the GPIO pins for the battery voltage measurement
 * analog input PC0 and PC1
 */
void Battery_GPIO_Init(void);

/**
 * @brief Initialize the GPIO pins for SPI2
 * SPI2: SCK (PB13), MISO (PB14), MOSI (PB15)
 * IMU CS (PB12), BARO CS (PC2), MAG CS (PD2)
 */
void SPI_Sensor_GPIO_Init(void);

/**
 * @brief Initialize the GPIO pins for PWM
 * TIM3 CH1 (PA6), CH2 (PA7), CH3 (PB0), CH4 (PB1)
 */
void PWM_GPIO_Init(void);

/**
 * @brief Initialize the GPIO pins for UART1
 * UART1: TX (PB6), RX (PB7)
 */
void UART1_GPIO_Init(void);

/**
 * @brief Initialize the GPIO pins for UART2
 * UART2: TX (PA2), RX (PA3)
 */
void UART2_GPIO_Init(void);

/**
 * @brief Initialize the GPIO pins for UART6
 * UART6: TX (PA11), RX (PA12)
 */
void UART6_GPIO_Init(void);

/**
 * @brief Initialize the GPIO pins for OLED
 * SCL1 (PB8) and SDA1 (PB9)
 */
void OLED_GPIO_Init(void);

#endif /* __BOARD_H */