/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "stdbool.h"


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	
		a1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
		b1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	
		a[Inc_red] = HAL_GPIO_ReadPin(IncA_port_red, IncA_pin_red);
		b[Inc_red] = HAL_GPIO_ReadPin(IncB_port_red, IncB_pin_red);
	
		a[Inc_aquamarin] = HAL_GPIO_ReadPin(IncA_port_aquamarin, IncA_pin_aquamarin);
		b[Inc_aquamarin] = HAL_GPIO_ReadPin(IncB_port_aquamarin, IncB_pin_aquamarin);
	
		a[Inc_black] = HAL_GPIO_ReadPin(IncA_port_black, IncA_pin_black);
		b[Inc_black] = HAL_GPIO_ReadPin(IncB_port_black, IncB_pin_black);
	
		a[Inc_blue] = HAL_GPIO_ReadPin(IncA_port_blue, IncA_pin_blue);
		b[Inc_blue] = HAL_GPIO_ReadPin(IncB_port_blue, IncB_pin_blue);
	
		a[Inc_yellow] = HAL_GPIO_ReadPin(IncA_port_yellow, IncA_pin_yellow);
		b[Inc_yellow] = HAL_GPIO_ReadPin(IncB_port_yellow, IncB_pin_yellow);
	
		a[Inc_green] = HAL_GPIO_ReadPin(IncA_port_green, IncA_pin_green);
		b[Inc_green] = HAL_GPIO_ReadPin(IncB_port_green, IncB_pin_green);
	
		a[Inc_clarity] = HAL_GPIO_ReadPin(IncA_port_clarity, IncA_pin_clarity);
		b[Inc_clarity] = HAL_GPIO_ReadPin(IncB_port_clarity, IncB_pin_clarity);
		
		a[Inc_contrast] = HAL_GPIO_ReadPin(IncA_port_contrast, IncA_pin_contrast);
		b[Inc_contrast] = HAL_GPIO_ReadPin(IncB_port_contrast, IncB_pin_contrast);
		
		a[Inc_crop] = HAL_GPIO_ReadPin(IncA_port_crop, IncA_pin_crop);
		b[Inc_crop] = HAL_GPIO_ReadPin(IncB_port_crop, IncB_pin_crop);
		
		a[Inc_dynamic] = HAL_GPIO_ReadPin(IncA_port_dynamic, IncA_pin_dynamic);
		b[Inc_dynamic] = HAL_GPIO_ReadPin(IncB_port_dynamic, IncB_pin_dynamic);
		
		a[Inc_exposure] = HAL_GPIO_ReadPin(IncA_port_exposure, IncA_pin_exposure);
		b[Inc_exposure] = HAL_GPIO_ReadPin(IncB_port_exposure, IncB_pin_exposure);
		
		a[Inc_lights] = HAL_GPIO_ReadPin(IncA_port_lights, IncA_pin_lights);
		b[Inc_lights] = HAL_GPIO_ReadPin(IncB_port_lights, IncB_pin_lights);
		
		a[Inc_magenta] = HAL_GPIO_ReadPin(IncA_port_magenta, IncA_pin_magenta);
		b[Inc_magenta] = HAL_GPIO_ReadPin(IncB_port_magenta, IncB_pin_magenta);
		
		a[Inc_orange] = HAL_GPIO_ReadPin(IncA_port_orange, IncA_pin_orange);
		b[Inc_orange] = HAL_GPIO_ReadPin(IncB_port_orange, IncB_pin_orange);
		
		a[Inc_prog] = HAL_GPIO_ReadPin(IncA_port_prog, IncA_pin_prog);
		b[Inc_prog] = HAL_GPIO_ReadPin(IncB_port_prog, IncB_pin_prog);
		
		a[Inc_saturation] = HAL_GPIO_ReadPin(IncA_port_saturation, IncA_pin_saturation);
		b[Inc_saturation] = HAL_GPIO_ReadPin(IncB_port_saturation, IncB_pin_saturation);
		
		a[Inc_shadow] = HAL_GPIO_ReadPin(IncA_port_shadow, IncA_pin_shadow);
		b[Inc_shadow] = HAL_GPIO_ReadPin(IncB_port_shadow, IncB_pin_shadow);
		
		a[Inc_violett] = HAL_GPIO_ReadPin(IncA_port_violett, IncA_pin_violett);
		b[Inc_violett] = HAL_GPIO_ReadPin(IncB_port_violett, IncB_pin_violett);
		
		a[Inc_white] = HAL_GPIO_ReadPin(IncA_port_white, IncA_pin_white);
		b[Inc_white] = HAL_GPIO_ReadPin(IncB_port_white, IncB_pin_white);
	
		if(timecount>0){
			timecount--;
		}
		
		
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
