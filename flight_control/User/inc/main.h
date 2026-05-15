/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/main.h
 * @author  MCD Application Team
 * @version V1.8.0
 * @date    04-November-2016
 * @brief   Header for main.c module
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __main_H
#define __main_H

/* Includes ------------------------------------------------------------------*/
#include "RTE_Components.h"
#include "stm32f4xx.h"

#include "fps.h"
#include "key.h"
#include "led.h"
#include "battery.h"
#include "ui.h"
#include "navigator.h"
#include "ble.h"
#include "dma.h"
#include "pwm.h"
#include "icm20948.h"
#include "bmp280.h"
// #include "mpu.h"
#include "attitude.h"
#include "persistence.h"
#include "spi_sensor.h"
#include "control.h"
#include "command.h"

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
