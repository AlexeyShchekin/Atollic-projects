/*
 * sd.c
 *
 *  Created on: 23 ���. 2019 �.
 *      Author: Alexey
 */
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "sd.h"

extern volatile uint16_t Timer1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

#define CMD0 (0x40+0) // GO_IDLE_STATE
#define CMD1 (0x40+1) // SEND_OP_COND (MMC)
#define ACMD41 (0xC0+41) // SEND_OP_COND (SDC)
#define CMD8 (0x40+8) // SEND_IF_COND
#define CMD9 (0x40+9) // SEND_CSD
#define CMD16 (0x40+16) // SET_BLOCKLEN
#define CMD17 (0x40+17) // READ_SINGLE_BLOCK
#define CMD24 (0x40+24) // WRITE_BLOCK
#define CMD55 (0x40+55) // APP_CMD
#define CMD58 (0x40+58) // READ_OCR

sd_info_ptr sdinfo;
char str1[60]={0};

static void SDError (void)
{
	LD_ON;
}

static uint8_t SD_cmd (uint8_t cmd, uint32_t arg)
{
	uint8_t n, res;
	if (cmd & 0x80)
	{
		cmd &= 0x7F;
		res = SD_cmd(CMD55, 0);
		if (res > 1) return res;
	}
	SS_SD_DESELECT();
	SPI_ReceiveByte();
	SS_SD_SELECT();
	SPI_ReceiveByte();

	SPI_SendByte(cmd); // Start + Command index
	SPI_SendByte((uint8_t)(arg >> 24)); // Argument[31..24]
	SPI_SendByte((uint8_t)(arg >> 16)); // Argument[23..16]
	SPI_SendByte((uint8_t)(arg >> 8)); // Argument[15..8]
	SPI_SendByte((uint8_t)arg); // Argument[7..0]

	n = 0x01; // Dummy CRC + Stop
	if (cmd == CMD0) {n = 0x95;} // Valid CRC for CMD0(0)
	if (cmd == CMD8) {n = 0x87;} // Valid CRC for CMD8(0x1AA)
	SPI_SendByte(n);
	n = 10; // Wait for a valid response in timeout of 10 attempts
	do
	{
		res = SPI_ReceiveByte();
	} while ((res & 0x80) && --n);
	return res;
}

void SD_PowerOn(void)
{
	Timer1 = 0;
	while(Timer1<2){}
}

uint8_t sd_ini(void)
{
	uint8_t i,cmd;
	int16_t tmr;
	uint8_t ocr[4];
	uint32_t temp;
	LD_OFF;
	sdinfo.type = 0;

	temp = hspi2.Init.BaudRatePrescaler;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; //156.25 kbbs
	HAL_SPI_Init(&hspi2);

	SS_SD_DESELECT();
	for(i=0;i<10;i++){
		SPI_Release();
	}

	hspi2.Init.BaudRatePrescaler = temp;
	HAL_SPI_Init(&hspi2);

	SS_SD_SELECT();
	if (SD_cmd(CMD0, 0) == 1) // Enter Idle state
	{
		SPI_Release();
		if (SD_cmd(CMD8, 0x1AA) == 1) // SDv2
		{
			for (i = 0; i < 4; i++)
			{
				ocr[i] = SPI_ReceiveByte();
			}
			sprintf(str1,"OCR: 0x%02X 0x%02X 0x%02X 0x%02Xrn",ocr[0],ocr[1],ocr[2],ocr[3]);
			HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);

			if (ocr[2] == 0x01 && ocr[3] == 0xAA) // The card can work at vdd range of 2.7-3.6V
			{
				for (tmr = 12000; tmr && SD_cmd(ACMD41, 1UL << 30); tmr--){}
				if (tmr && SD_cmd(CMD58, 0) == 0)
				{ // Check CCS bit in the OCR
					for (i = 0; i < 4; i++) {ocr[i] = SPI_ReceiveByte();}
					sprintf(str1,"OCR: 0x%02X 0x%02X 0x%02X 0x%02Xrn",ocr[0],ocr[1],ocr[2],ocr[3]);
					HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
					sdinfo.type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; // SDv2 (HC or SC)
				}
			}
		}
		else //SDv1 or MMCv3
		{
			if (SD_cmd(ACMD41, 0) <= 1)
			{
				sdinfo.type = CT_SD1; cmd = ACMD41; // SDv1
			}
			else
			{
				sdinfo.type = CT_MMC; cmd = CMD1; // MMCv3
			}
			for (tmr = 25000; tmr && SD_cmd(cmd, 0); tmr--) {} // Wait for leaving idle state
			if (!tmr || SD_cmd(CMD16, 512) != 0) // Set R/W block length to 512
			{
				sdinfo.type = 0;
			}
		}
	}
	else
	{
		return 1;
	}
	sprintf(str1,"Type SD: 0x%02Xrn",sdinfo.type);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	return 0;
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		SDError();
	}
	return receivedbyte;
}

void SPI_SendByte(uint8_t bt)
{
	SPIx_WriteRead(bt);
}

uint8_t SPI_ReceiveByte(void)
{
	uint8_t bt = SPIx_WriteRead(0xFF);
	return bt;
}

void SPI_Release(void)
{
	SPIx_WriteRead(0xFF);
}