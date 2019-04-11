/*
 * ili9341.c
 *
 *  Created on: 11 ���. 2019 �.
 *      Author: Alexey
 */

#include "ili9341.h"

uint16_t X_SIZE = 0;
uint16_t Y_SIZE = 0;
uint32_t dtt = 0;
extern RNG_HandleTypeDef hrng;

void TFT9341_Delay(uint32_t dly)
{
   uint32_t i;
   for(i = 0; i < dly; i++);
}

__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
       micros *=(SystemCoreClock / 1000000) / 5;
       while (micros--);
}

void TFT9341_ini(void)
{
	//char str[10];
	TFT9341_reset();
	HAL_Delay(1000);
	dtt = TFT9341_ReadReg(0xD3);
	TFT9341_SendCommand(0x01);//Software Reset
	DelayMicro(1);

	TFT9341_SendCommand(0xCB);//Power Control A
	TFT9341_SendData(0x39);
	TFT9341_SendData(0x2C);
	TFT9341_SendData(0x00);
	TFT9341_SendData(0x34);
	TFT9341_SendData(0x02);
	DelayMicro(1);

	TFT9341_SendCommand(0xCF);//Power Control B
	TFT9341_SendData(0x00);
	TFT9341_SendData(0xC1);
	TFT9341_SendData(0x30);
	DelayMicro(1);

	TFT9341_SendCommand(0xE8);//Driver timing control A
	TFT9341_SendData(0x85);
	TFT9341_SendData(0x00);
	TFT9341_SendData(0x78);
	DelayMicro(1);

	TFT9341_SendCommand(0xEA);//Driver timing control B
	TFT9341_SendData(0x00);
	TFT9341_SendData(0x00);
	DelayMicro(1);

	TFT9341_SendCommand(0xED);//Power on Sequence control
	TFT9341_SendData(0x64);
	TFT9341_SendData(0x03);
	TFT9341_SendData(0x12);
	TFT9341_SendData(0x81);
	DelayMicro(1);

	TFT9341_SendCommand(0xF7);//Pump ratio control
	TFT9341_SendData(0x20);
	DelayMicro(1);

	TFT9341_SendCommand(0xC0);//Power Control 1
	TFT9341_SendData(0x10);
	DelayMicro(1);

	TFT9341_SendCommand(0xC1);//Power Control 2
	TFT9341_SendData(0x10);
	DelayMicro(1);

	TFT9341_SendCommand(0xC5);//VCOM Control 1
	TFT9341_SendData(0x3E);
	TFT9341_SendData(0x28);
	DelayMicro(1);

	TFT9341_SendCommand(0xC7);//VCOM Control 2
	TFT9341_SendData(0x86);
	DelayMicro(1);

	TFT9341_SetRotation(0);
	DelayMicro(1);

	TFT9341_SendCommand(0x3A);//Pixel Format Set
	TFT9341_SendData(0x55);//16bit
	DelayMicro(1);

	TFT9341_SendCommand(0xB1);
	TFT9341_SendData(0x00);
	TFT9341_SendData(0x18);// ������� ������ 79 ��
	DelayMicro(1);

	TFT9341_SendCommand(0xB6);//Display Function Control
	TFT9341_SendData(0x08);
	TFT9341_SendData(0x82);
	TFT9341_SendData(0x27);//320 �����
	DelayMicro(1);

	TFT9341_SendCommand(0xF2);//Enable 3G (���� �� ���� ��� ��� �� �����)
	TFT9341_SendData(0x00);//�� ��������
	DelayMicro(1);

	TFT9341_SendCommand(0x26);//Gamma set
	TFT9341_SendData(0x01);//Gamma Curve (G2.2) (������ �������� �����)
	DelayMicro(1);

	TFT9341_SendCommand(0xE0);//Positive Gamma  Correction
	TFT9341_SendData(0x0F);
	TFT9341_SendData(0x31);
	TFT9341_SendData(0x2B);
	TFT9341_SendData(0x0C);
	TFT9341_SendData(0x0E);
	TFT9341_SendData(0x08);
	TFT9341_SendData(0x4E);
	TFT9341_SendData(0xF1);
	TFT9341_SendData(0x37);
	TFT9341_SendData(0x07);
	TFT9341_SendData(0x10);
	TFT9341_SendData(0x03);
	TFT9341_SendData(0x0E);
	TFT9341_SendData(0x09);
	TFT9341_SendData(0x00);
	DelayMicro(1);

	TFT9341_SendCommand(0xE1);//Negative Gamma  Correction
	TFT9341_SendData(0x00);
	TFT9341_SendData(0x0E);
	TFT9341_SendData(0x14);
	TFT9341_SendData(0x03);
	TFT9341_SendData(0x11);
	TFT9341_SendData(0x07);
	TFT9341_SendData(0x31);
	TFT9341_SendData(0xC1);
	TFT9341_SendData(0x48);
	TFT9341_SendData(0x08);
	TFT9341_SendData(0x0F);
	TFT9341_SendData(0x0C);
	TFT9341_SendData(0x31);
	TFT9341_SendData(0x36);
	TFT9341_SendData(0x0F);
	DelayMicro(1);

	TFT9341_SendCommand(0x11);//������ �� ������� �����
	HAL_Delay(150);
	TFT9341_SendCommand(0x29);//��������� �������
	TFT9341_SendData(0x2C);
	HAL_Delay(150);
}

void TFT9341_SendCommand(unsigned char cmd)
{
       ADDR_CMD = cmd;
}

void TFT9341_SendData(unsigned char dt)
{
       ADDR_DATA = dt;
       //TFT9341_Delay(1);
       DelayMicro(1);
}

uint32_t TFT9341_ReadReg(uint8_t r)
{
	uint32_t id;
	uint8_t x;
	TFT9341_SendCommand(r);
	DelayMicro(50);
	x=ADDR_DATA;
	id=x;
	id<<=8;
	DelayMicro(1);
	x=ADDR_DATA;
	id|=x;
	id<<=8;
	DelayMicro(1);
	x=ADDR_DATA;
	id|=x;
	id<<=8;
	DelayMicro(1);
	x=ADDR_DATA;
	id|=x;

	if(r==0xEF)
	{
		id<<=8;
		DelayMicro(5);
		x=ADDR_DATA;
		id|=x;
	}
	DelayMicro(150);//stabilization time
	return id;
}

void TFT9341_SetRotation(unsigned char r)
{
	TFT9341_SendCommand(0x36);
    switch(r)
    {
		case 0:
			TFT9341_SendData(0x48);
			X_SIZE = 240;
			Y_SIZE = 320;
		break;
		case 1:
			TFT9341_SendData(0x28);
			X_SIZE = 320;
			Y_SIZE = 240;
		break;
		case 2:
			TFT9341_SendData(0x88);
			X_SIZE = 240;
			Y_SIZE = 320;
		break;
		case 3:
			TFT9341_SendData(0xE8);
			X_SIZE = 320;
			Y_SIZE = 240;
		break;
    }
}

void TFT9341_reset(void)
{
	RESET_ACTIVE;
	HAL_Delay(2);
	RESET_IDLE;
	TFT9341_SendCommand(0x01); //Software Reset
	for (uint8_t i=0;i<3;i++)
	{
		TFT9341_SendData(0xFF);
	}
}

uint16_t TFT9341_RandColor(void)
{
   return HAL_RNG_GetRandomNumber(&hrng)&0x0000FFFF;
}

void TFT9341_Flood(uint16_t color, uint32_t len)
{
	uint16_t blocks;
	uint8_t i, hi = color>>8, lo=color;
	TFT9341_SendCommand(0x2C);
	TFT9341_SendData(hi);
	DelayMicro(1);
	TFT9341_SendData(lo);
	len--;
	blocks=(uint16_t)(len/64);//64 pixels/block
	while(blocks--)
	{
	   i=16;
	   do
	   {
		   TFT9341_SendData(hi);
		   TFT9341_SendData(lo);
		   TFT9341_SendData(hi);
		   TFT9341_SendData(lo);
		   TFT9341_SendData(hi);
		   TFT9341_SendData(lo);
		   TFT9341_SendData(hi);
		   TFT9341_SendData(lo);
	   } while (--i);
	}
	//Fill any remaining pixels(1 to 64)
	for (i=(uint8_t)len&63;i--;)
	{
	   TFT9341_SendData(hi);
	   TFT9341_SendData(lo);
	}
}

void TFT9341_SetAddrWindow(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	TFT9341_SendCommand(0x2A);//Column Addres Set
	TFT9341_SendData(x1 >> 8);
	TFT9341_SendData(x1 & 0xFF);
	TFT9341_SendData(x2 >> 8);
	TFT9341_SendData(x2 & 0xFF);
	DelayMicro(1);

	TFT9341_SendCommand(0x2B);//Page Addres Set
	TFT9341_SendData(y1 >> 8);
	TFT9341_SendData(y1 & 0xFF);
	TFT9341_SendData(y2 >> 8);
	TFT9341_SendData(y2 & 0xFF);
	DelayMicro(1);
}

void TFT9341_FillScreen(uint16_t color)
{
	TFT9341_SetAddrWindow(0,0,X_SIZE-1,Y_SIZE-1);
	TFT9341_Flood(color,(long)X_SIZE*(long)Y_SIZE);
}

void TFT9341_FillRectangle(uint16_t color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
   TFT9341_SetAddrWindow(x1, y1, x2, y2);
   TFT9341_Flood(color, (uint16_t)(x2-x1+1) * (uint16_t)(y2-y1+1));
}
