/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

#include "main.h"

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
  // vPortSVCHandler();
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
  // xPortPendSVHandler();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    systemTick++;
    // xPortSysTickHandler();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/


void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
        volatile uint32_t temp;
        temp = USART1->SR;
        temp = USART1->DR;

        DMA_Cmd(DMA2_Stream2, DISABLE); 
        uint16_t timeout;
        for (timeout = 0xFFFF; DMA_GetCmdStatus(DMA2_Stream2) != DISABLE && timeout > 0; timeout--) ;

        bleRxIndexUart1 = sizeof(bleRxBuffer) - DMA_GetCurrDataCounter(DMA2_Stream2);
        bleRxStatusUart1 = BLE_RX_STATE_COMPLETE;

        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_FEIF2);

        DMA_SetCurrDataCounter(DMA2_Stream2, sizeof(bleRxBuffer));
        DMA_Cmd(DMA2_Stream2, ENABLE);
    }
}

void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
        // 清除 IDLE 标志
        volatile uint32_t temp;
        temp = USART2->SR;
        temp = USART2->DR;

        esRxStatusUart2 = ES_RX_STATE_COMPLETE;
    }

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        LED_On();
        if (esRxIndexUart2 < ES_RX_BUFFER_SIZE - 1) {
            esRxBuffer[esRxIndexUart2++] = (uint8_t)USART_ReceiveData(USART2);
        }
    }
}

void DMA1_Stream6_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6)) {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
        I2C_GenerateSTOP(I2C1, ENABLE);
        DMA_Cmd(DMA1_Stream6, DISABLE);
    }
}


void ADC_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC))
    {
        pwr_adc_raw = ADC_GetConversionValue(ADC1);
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
        pwr_state = PWR_STATE_DISABLE;
    }
}


void TIM4_IRQHandler(void) {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        ControlMotor_Loop();
    }
}

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        if (spi2Occupied == 0) {
            spi2Occupied = 1;
            GPIO_ResetBits(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);

            DMA_Cmd(DMA1_Stream3, ENABLE);
            DMA_Cmd(DMA1_Stream4, ENABLE);
        }
    }
}

void DMA1_Stream3_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3)) {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
        
        GPIO_SetBits(GPIO_IMU_SPI, GPIO_IMU_SPI_CS_PIN);

        // 重置 DMA 计数器配合 Normal 模式使用，防止时序错位
        DMA_Cmd(DMA1_Stream3, DISABLE);
        DMA_Cmd(DMA1_Stream4, DISABLE);

        while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
        while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE);

        DMA_SetCurrDataCounter(DMA1_Stream3, 24);
        DMA_SetCurrDataCounter(DMA1_Stream4, 24);

        spi2Occupied = 0;
    }
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */


//void TIM3_IRQHandler(void)
//{
//  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
//  {
//    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//    TIM3_IT_Update_Callback();
//  }
//  
//}

 
/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


