#ifndef __NAVIGATOR_H
#define __NAVIGATOR_H

#include <stdio.h>

#include "stm32f4xx.h"
#include "ui.h"

#include "icm20948.h"
#include "bmp280.h"

#include "attitude.h"
#include "fps.h"
#include "battery.h"
#include "control.h"
// #include "mpu.h"

#define WINDOW_NONE              0
#define WINDOW_HOME              1
#define WINDOW_IMU               2
#define WINDOW_PID               3
#define WINDOW_ACCEL_CALIBRATE   4
#define WINDOW_GYRO_CALIBRATE    5
#define WINDOW_MAG_CALIBRATE     6   
#define WINDOW_CUBE              7
#define WINDOW_BATTERY           8

extern UI_Logger logWindow;

void Window_Init(void);

uint8_t Window_Current(void);
void Window_To(uint8_t windowId);

void Window_Render(void);


#endif /* __NAVIGATOR_H */