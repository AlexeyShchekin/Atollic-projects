/*
 * DPS310.c
 *
 *  Created on: Dec 25, 2017
 *      Author: a.shchekin
 */

#include "DPS310.h"

//const int32_t Dps310_Scaling_facts[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

#define PM_RATE_4 0x10
#define PM_PRC_64 0x07

int32_t kP = 1040384;
int32_t kT = 524288;

void ReadCoeffs(uint16_t device_addr, int * coeffs)
{
	uint8_t cmd[2] = {0x0C, 0x89};
	HAL_Delay(12);
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2); // Software Reset, FIFO-Flush
	HAL_Delay(40);

	uint8_t temp;
	do
	{
		i2cSoft_WriteCMD(device_addr<<1, 0x08);
		i2cSoft_ReadBuffer(device_addr<<1, &temp, 1);
	} while (((temp>>4)&0x0C) != 0x0C);	//Sensor is ready, coeffs are ready

	uint8_t res[18];
	uint8_t reg_addr;

	for (uint8_t i=0; i<18; i++)
	{
		reg_addr = 0x10 + i;
		i2cSoft_WriteCMD(device_addr<<1, reg_addr);
		i2cSoft_ReadBuffer(device_addr<<1, &(temp), 1);
		res[i] = temp;
	}

	coeffs[0] = ((uint32_t)res[0] << 4) | (((uint32_t)res[1] >> 4) & 0x0F);
	if (coeffs[0] > ((1U << 11) - 1))
	{
		coeffs[0] = coeffs[0] - (1U << 12);
	}

	coeffs[1] = ((uint32_t)(res[1]&0x0F) << 8) | ((uint32_t)res[2]);
	if (coeffs[1] > ((1U << 11) - 1))
	{
		coeffs[1] = coeffs[1] - (1U << 12);
	}

	coeffs[2] = ((uint32_t)res[3] << 12) | ((uint32_t)res[4] << 4) | (((uint32_t)res[5] >> 4) & 0x0F);
	if (coeffs[2] > ((1U << 19) - 1))
	{
		coeffs[2] = coeffs[2] - (1U << 20);
	}

	coeffs[3] = ((uint32_t)(res[5]&0x0F) << 16) | ((uint32_t)res[6] << 8) | ((uint32_t)res[7]);
	if (coeffs[3] > ((1U << 19) - 1))
	{
		coeffs[3] = coeffs[3] - (1U << 20);
	}

	for (uint8_t i=0x00; i<0x05; i++)
	{
		coeffs[4+i] = ((uint32_t)res[8+2*i] << 8) | ((uint32_t)res[9+2*i]);
		if (coeffs[4+i] > ((1U << 15) - 1))
		{
			coeffs[4+i] = coeffs[4+i] - (1U << 16);
		}
	}

	cmd[0] = 0x0E;
	cmd[1] = 0xA5;
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);
	cmd[0] = 0x0F;
	cmd[1] = 0x96;
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);
	cmd[0] = 0x62;
	cmd[1] = 0x02;
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);
	cmd[0] = 0x0E;
	cmd[1] = 0x00;
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);
	cmd[0] = 0x0F;
	cmd[1] = 0x00;
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);

	HAL_Delay(100);
}


void CFG_Measurement(uint16_t device_addr, uint8_t PRECISION)
{
	uint8_t cmd[2] = {0x06, 0x26};
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);

	uint8_t temp_sens;
	i2cSoft_WriteCMD(device_addr<<1, 0x28);
	i2cSoft_ReadBuffer(device_addr<<1, &(temp_sens), 1);

	cmd[0] = 0x07;
	cmd[1] = 0xA0;
	if (((temp_sens>>7)&0x01)==0x01)
	{
		cmd[1] = cmd[1] | (1<<7);
	}
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);

	cmd[0] = 0x09;
	cmd[1] = 0b00000100;
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);

	/*cmd[0] = 0x08;
	cmd[1] = 0xC7;	//Continuous
	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);*/
}

void GetReadings(uint16_t device_addr, int * coeffs, double * Readings)
{
	uint8_t res = 0x00;
	do
	{
		i2cSoft_WriteCMD(device_addr<<1, 0x08);
		i2cSoft_ReadBuffer(device_addr<<1, &res, 1);
	} while (((res>>4)&0x0F) != 0x0F);

	uint8_t buffer[3] = {0};
	i2cSoft_WriteCMD(device_addr<<1, 0x00);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[0], 1);
	i2cSoft_WriteCMD(device_addr<<1, 0x01);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[1], 1);
	i2cSoft_WriteCMD(device_addr<<1, 0x02);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[1], 1);
	int32_t Praw = ((uint32_t)buffer[0] << 16)	| ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
	if (Praw & ((uint32_t)1 << 23))
	{
		Praw -= (uint32_t)1 << 24;
	}

	i2cSoft_WriteCMD(device_addr<<1, 0x03);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[0], 1);
	i2cSoft_WriteCMD(device_addr<<1, 0x04);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[1], 1);
	i2cSoft_WriteCMD(device_addr<<1, 0x05);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[1], 1);
	int32_t Traw = ((uint32_t)buffer[0] << 16)	| ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
	if(Traw & ((uint32_t)1 << 23))
	{
		Traw -= (uint32_t)1 << 24;
	}

	double Tsc = (double)Traw/((double)kT);
	double Psc = (double)Praw/((double)kP);
	Readings[0] = coeffs[0]*0.5 + coeffs[1]*Tsc;
	Readings[1] = coeffs[2] + Psc*(coeffs[3] + Psc*(coeffs[6] + Psc*coeffs[8])) + Tsc*coeffs[4] + Tsc*Psc*(coeffs[5] + Psc*coeffs[7]);
}

int32_t GetTemperature(uint16_t device_addr)
{
	uint8_t cmd[2] = {0x08, 0x02};
	uint8_t res = 0x00;

	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);

	do
	{
		i2cSoft_WriteCMD(device_addr<<1, cmd[0]);
		i2cSoft_ReadBuffer(device_addr<<1, &res, 1);
	} while ((res&0x20) != 0x20);

	uint8_t buffer[3] = {0};
	i2cSoft_WriteCMD(device_addr<<1, 0x00);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[0], 1);
	i2cSoft_WriteCMD(device_addr<<1, 0x01);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[1], 1);
	i2cSoft_WriteCMD(device_addr<<1, 0x02);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[2], 1);
	int32_t Traw = ((uint32_t)buffer[0] << 16)	| ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
	if (Traw & ((uint32_t)1 << 23))
	{
		Traw -= (uint32_t)1 << 24;
	}
	return Traw;
}

int32_t GetPressure(uint16_t device_addr)
{
	uint8_t cmd[2] = {0x08, 0x01};
	uint8_t res = 0x00;

	i2cSoft_WriteBuffer(device_addr<<1, &cmd[0], 2);

	do
	{
		i2cSoft_WriteCMD(device_addr<<1, cmd[0]);
		i2cSoft_ReadBuffer(device_addr<<1, &res, 1);
	} while ((res&0x10) != 0x10);

	uint8_t buffer[3] = {0};
	i2cSoft_WriteCMD(device_addr<<1, 0x00);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[0], 1);
	i2cSoft_WriteCMD(device_addr<<1, 0x01);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[1], 1);
	i2cSoft_WriteCMD(device_addr<<1, 0x02);
	i2cSoft_ReadBuffer(device_addr<<1, &buffer[2], 1);
	int32_t Praw = ((uint32_t)buffer[0] << 16)	| ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
	if (Praw & ((uint32_t)1 << 23))
	{
		Praw -= (uint32_t)1 << 24;
	}
	return Praw;
}

double CalibTemperature(int32_t Traw, int * coeffs)
{
	double Tsc = (double)Traw/((double)kT);
	return coeffs[0]*0.5 + coeffs[1]*Tsc;
}

double CalibPressure(int32_t Traw, int32_t Praw, int * coeffs)
{
	double Tsc = (double)Traw/((double)kT);
	double Psc = (double)Praw/((double)kP);
	return coeffs[2] + Psc*(coeffs[3] + Psc*(coeffs[6] + Psc*coeffs[8])) + Tsc*coeffs[4] + Tsc*Psc*(coeffs[5] + Psc*coeffs[7]);
}
