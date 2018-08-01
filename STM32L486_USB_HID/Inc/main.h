/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DB0_Pin GPIO_PIN_0
#define DB0_GPIO_Port GPIOC
#define DB1_Pin GPIO_PIN_1
#define DB1_GPIO_Port GPIOC
#define DB2_Pin GPIO_PIN_2
#define DB2_GPIO_Port GPIOC
#define DB3_Pin GPIO_PIN_3
#define DB3_GPIO_Port GPIOC
#define DB4_Pin GPIO_PIN_4
#define DB4_GPIO_Port GPIOC
#define DB5_Pin GPIO_PIN_5
#define DB5_GPIO_Port GPIOC
#define OS0_Pin GPIO_PIN_0
#define OS0_GPIO_Port GPIOB
#define OS1_Pin GPIO_PIN_1
#define OS1_GPIO_Port GPIOB
#define OS2_Pin GPIO_PIN_2
#define OS2_GPIO_Port GPIOB
#define DB6_Pin GPIO_PIN_6
#define DB6_GPIO_Port GPIOC
#define DB7_Pin GPIO_PIN_7
#define DB7_GPIO_Port GPIOC
#define BUSY_Pin GPIO_PIN_8
#define BUSY_GPIO_Port GPIOC
#define FRSTDATA_Pin GPIO_PIN_9
#define FRSTDATA_GPIO_Port GPIOC
#define PARSEL_Pin GPIO_PIN_3
#define PARSEL_GPIO_Port GPIOB
#define CONVSTA_Pin GPIO_PIN_4
#define CONVSTA_GPIO_Port GPIOB
#define CONVSTB_Pin GPIO_PIN_5
#define CONVSTB_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_6
#define RESET_GPIO_Port GPIOB
#define RD_Pin GPIO_PIN_7
#define RD_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_8
#define CS_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define CS_L (CS_GPIO_Port->BRR = CS_Pin)
#define CS_H (CS_GPIO_Port->BSRR = CS_Pin)
#define RD_L (RD_GPIO_Port->BRR = RD_Pin)
#define RD_H (RD_GPIO_Port->BSRR = RD_Pin)
#define OS0_L (OS0_GPIO_Port->BRR = OS0_Pin)
#define OS0_H (OS0_GPIO_Port->BSRR = OS0_Pin)
#define OS1_L (OS1_GPIO_Port->BRR = OS1_Pin)
#define OS1_H (OS1_GPIO_Port->BSRR = OS1_Pin)
#define OS2_L (OS2_GPIO_Port->BRR = OS2_Pin)
#define OS2_H (OS2_GPIO_Port->BSRR = OS2_Pin)
#define PARSEL_L (PARSEL_GPIO_Port->BRR = PARSEL_Pin)
#define PARSEL_H (PARSEL_GPIO_Port->BSRR = PARSEL_Pin)
#define CONVSTA_L (CONVSTA_GPIO_Port->BRR = CONVSTA_Pin)
#define CONVSTA_H (CONVSTA_GPIO_Port->BSRR = CONVSTA_Pin)
#define CONVSTB_L (CONVSTB_GPIO_Port->BRR = CONVSTB_Pin)
#define CONVSTB_H (CONVSTB_GPIO_Port->BSRR = CONVSTB_Pin)
#define RESET_L (RESET_GPIO_Port->BRR = RESET_Pin)
#define RESET_H (RESET_GPIO_Port->BSRR = RESET_Pin)
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
