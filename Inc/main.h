/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
extern volatile bool a1;
extern volatile bool b1;

extern volatile bool a[19];
extern volatile bool b[19];

extern volatile unsigned int timecount;

#define TIMECOUNT 500 //Speed measurement encoder
#define Time1		200
#define Time2		300
#define Speed1	1 //Faktor 1
#define Speed2	2 //Faktor 2
#define Speed3	4 //Faktor 4

#define Inc_red					0
#define Inc_orange			1
#define Inc_yellow			2
#define Inc_green				3
#define Inc_aquamarin		4
#define Inc_blue				5
#define Inc_violett			6
#define Inc_magenta			7
#define Inc_contrast		8
#define Inc_exposure		9
#define Inc_shadow			10
#define Inc_lights			11
#define Inc_black				12
#define Inc_white				13
#define Inc_clarity			14
#define Inc_dynamic			15
#define Inc_saturation	16
#define Inc_prog				17
#define Inc_crop				18


#define IncA_pin_red					GPIO_PIN_15
#define IncB_pin_red					GPIO_PIN_0
#define IncA_pin_orange				GPIO_PIN_7
#define IncB_pin_orange				GPIO_PIN_6
#define IncA_pin_yellow				GPIO_PIN_9
#define IncB_pin_yellow				GPIO_PIN_10
#define IncA_pin_green				GPIO_PIN_8
#define IncB_pin_green				GPIO_PIN_9
#define IncA_pin_aquamarin		GPIO_PIN_8
#define IncB_pin_aquamarin		GPIO_PIN_7
#define IncA_pin_blue					GPIO_PIN_14
#define IncB_pin_blue					GPIO_PIN_13
#define IncA_pin_violett			GPIO_PIN_12
#define IncB_pin_violett			GPIO_PIN_11
#define IncA_pin_magenta			GPIO_PIN_4
#define IncB_pin_magenta			GPIO_PIN_7
#define IncA_pin_contrast			GPIO_PIN_13
#define IncB_pin_contrast			GPIO_PIN_14
#define IncA_pin_exposure			GPIO_PIN_2
#define IncB_pin_exposure			GPIO_PIN_3
#define IncA_pin_shadow				GPIO_PIN_5
#define IncB_pin_shadow				GPIO_PIN_4
#define IncA_pin_lights				GPIO_PIN_12
#define IncB_pin_lights				GPIO_PIN_2
#define IncA_pin_black				GPIO_PIN_1
#define IncB_pin_black				GPIO_PIN_2
#define IncA_pin_white				GPIO_PIN_12
#define IncB_pin_white				GPIO_PIN_11
#define IncA_pin_clarity			GPIO_PIN_6
#define IncB_pin_clarity			GPIO_PIN_15
#define IncA_pin_dynamic			GPIO_PIN_10
#define IncB_pin_dynamic			GPIO_PIN_5
#define IncA_pin_saturation		GPIO_PIN_4
#define IncB_pin_saturation		GPIO_PIN_3
#define IncA_pin_prog					GPIO_PIN_6
#define IncB_pin_prog					GPIO_PIN_5
#define IncA_pin_crop					GPIO_PIN_0
#define IncB_pin_crop					GPIO_PIN_1


#define IncA_port_red					GPIOC
#define IncB_port_red					GPIOC
#define IncA_port_orange			GPIOB
#define IncB_port_orange			GPIOB
#define IncA_port_yellow			GPIOA
#define IncB_port_yellow			GPIOA
#define IncA_port_green				GPIOA
#define IncB_port_green				GPIOC
#define IncA_port_aquamarin		GPIOC
#define IncB_port_aquamarin		GPIOC
#define IncA_port_blue				GPIOB
#define IncB_port_blue				GPIOB
#define IncA_port_violett			GPIOB
#define IncB_port_violett			GPIOB
#define IncA_port_magenta			GPIOC
#define IncB_port_magenta			GPIOA
#define IncA_port_contrast		GPIOC
#define IncB_port_contrast		GPIOC
#define IncA_port_exposure		GPIOA
#define IncB_port_exposure		GPIOC
#define IncA_port_shadow			GPIOB
#define IncB_port_shadow			GPIOB
#define IncA_port_lights			GPIOC
#define IncB_port_lights			GPIOD
#define IncA_port_black				GPIOC
#define IncB_port_black				GPIOC
#define IncA_port_white				GPIOA
#define IncB_port_white				GPIOA
#define IncA_port_clarity			GPIOC
#define IncB_port_clarity			GPIOB
#define IncA_port_dynamic 		GPIOB
#define IncB_port_dynamic 		GPIOC
#define IncA_port_saturation 	GPIOA
#define IncB_port_saturation 	GPIOA
#define IncA_port_prog 				GPIOA
#define IncB_port_prog 				GPIOA
#define IncA_port_crop 				GPIOA
#define IncB_port_crop 				GPIOA

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
