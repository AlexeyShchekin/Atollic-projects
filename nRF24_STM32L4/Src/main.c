
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "NRF24.h"
#include <string.h>
#include "DWT_us.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint16_t ADC_BUF[2];
extern ADC_HandleTypeDef hadc1;
char str1[32]={0};
uint8_t buf1[32]={0};
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ADC_BUF[0]);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ADC_BUF[1]);
}*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t dt_reg=0;
	uint8_t retr_cnt, dt;
	uint16_t i=1,retr_cnt_full;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Delay_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  NRF24_ini();
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&ADC_BUF,2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t status=0x00, regval=0x00;
	NRF24L01_TX_Mode(buf1);
	regval = NRF24_ReadReg(CONFIG);
	regval |= (1<<PWR_UP);
	regval &= ~(1<<PRIM_RX);
	NRF24_WriteReg(CONFIG,regval);
	DelayMicro(150);
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //HAL_Delay(100);

	   /* dt_reg = NRF24_ReadReg(CONFIG);

	    sprintf(str1,"CONFIG: 0x%02Xrn",dt_reg);

	    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t *)str1, strlen(str1));

	    HAL_Delay(10);

	    dt_reg = NRF24_ReadReg(EN_AA);

	    sprintf(str1,"EN_AA: 0x%02Xrn",dt_reg);

	    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t *)str1, strlen(str1));

	    HAL_Delay(10);

	    dt_reg = NRF24_ReadReg(EN_RXADDR);

	    sprintf(str1,"EN_RXADDR: 0x%02Xrn",dt_reg);

	    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t *)str1, strlen(str1));

	    HAL_Delay(10);

	    dt_reg = NRF24_ReadReg(STATUS);

	    sprintf(str1,"STATUS: 0x%02Xrn",dt_reg);

	    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t *)str1, strlen(str1));

	    HAL_Delay(10);

	    dt_reg = NRF24_ReadReg(RF_SETUP);

	    sprintf(str1,"RF_SETUP: 0x%02Xrn",dt_reg);

	    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t *)str1, strlen(str1));

	    HAL_Delay(10);

	    NRF24_Read_Buf(TX_ADDR,buf1,3);

	    sprintf(str1,"TX_ADDR: 0x%02X, 0x%02X, 0x%02Xrn",buf1[0],buf1[1],buf1[2]);

	    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t *)str1, strlen(str1));

	    HAL_Delay(10);

	    NRF24_Read_Buf(RX_ADDR_P0,buf1,3);

	    sprintf(str1,"RX_ADDR: 0x%02X, 0x%02X, 0x%02Xrn",buf1[0],buf1[1],buf1[2]);

	    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t *)str1, strlen(str1));

	    HAL_Delay(10);

	    	    sprintf(str1,"ADC: 0x%02X, 0x%02Xrn",ADC_BUF[0],ADC_BUF[1]);

	    	    HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	    	    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(uint8_t *)str1, strlen(str1));*/

	    //HAL_Delay(10);
	    //dt = NRF24L01_Send((uint8_t *)ADC_BUF);

	    buf1[0] = ADC_BUF[0]&0xFF;
	    buf1[1] = (ADC_BUF[0]>>8)&0xFF;
	    buf1[2] = ADC_BUF[1]&0xFF;
	    buf1[3] = (ADC_BUF[1]>>8)&0xFF;
	    //memcpy(buf1,(uint8_t*)&ADC_BUF[0],2);
	    //memcpy(buf1+2,(uint8_t*)&ADC_BUF[1],2);
	    //dt = NRF24L01_Send(buf1);

	    NRF24_Transmit(WR_TX_PLOAD, buf1, 5);
	      CE_SET;
	      DelayMicro(15); //minimum 10us high pulse (Page 21)
	      CE_RESET;

	      while((GPIO_PinState)IRQ == GPIO_PIN_SET) {}
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
