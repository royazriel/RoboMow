/**
  ******************************************************************************
  * @file    IO_Toggle/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
#include "stm32f0xx_it.h"
#include "hardware_config.h"
#include "stm32f0xx_exti.h"

__IO unsigned long time32_incr;

/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
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
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	time32_incr++;
#ifdef FILESYSTEM_ENABLE
	fsd_service();
#endif
}

void EXTI0_1_IRQHandler(void)
{
//	if(EXTI_GetITStatus(DECAIRQ_EXTI) != RESET)
//	{
//		printUSART("got it");
//	    process_deca_irq();
//	    led_on(LED_PC6);
//	    /* Clear the EXTI line 0 pending bit */
//	    EXTI_ClearITPendingBit(DECAIRQ_EXTI);
//	}
}

void EXTI4_15_IRQHandler(void)
{

}

void DMA1_Channel1_IRQHandler(void)
{
	UsartPrintf("DMA1_channel1 interrupt\r\n");
}


/**
  * @brief  This function handles ADC interrupt request.
  *         It manages the ADC in case of overrun
  *         the ADC is stopped but not disabled,
  * @param  None
  * @retval None
  */
void ADC1_COMP_IRQHandler(void)
{
	UsartPrintf("ADC1 interrupt\r\n");
}

DMA1_Channel2_3_IRQHandler(void)
{
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, DISABLE);
	WavPlayerSwapBuffers();
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
}
/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
