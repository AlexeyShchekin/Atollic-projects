/*
 * ili9341.h
 *
 *  Created on: 11 ���. 2019 �.
 *      Author: Alexey
 */

#ifndef ILI9341_H_
#define ILI9341_H_

#include <stdlib.h>
#include "stm32f7xx_hal.h"
#include "main.h"

#define ADDR_CMD        *(uint8_t*)0x60000000
#define ADDR_DATA       *(uint8_t*)0x60010000

#define swap(a,b) {int16_t t=a;a=b;b=t;}

#define  RESET_ACTIVE   HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
#define  RESET_IDLE   	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);

#define  BLACK 0x0000
#define  BLUE 0x001F
#define  RED 0x0F800
#define  GREEN 0x07E0
#define  CYAN 0x07FF
#define  MAGENTA 0xF81F
#define  YELLOW 0xFFE0
#define  WHITE 0xFFFF

void TFT9341_ini(void);
void TFT9341_SendCommand(unsigned char cmd);
void TFT9341_SendData(unsigned char dt);
uint32_t TFT9341_ReadReg(uint8_t r);
void TFT9341_SetRotation(unsigned char r);
void TFT9341_reset(void);
uint16_t TFT9341_RandColor(void);
void TFT9341_Flood(uint16_t color, uint32_t len);
void TFT9341_SetAddrWindow(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void TFT9341_FillScreen(uint16_t color);
void TFT9341_FillRectangle(uint16_t color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

#endif /* ILI9341_H_ */