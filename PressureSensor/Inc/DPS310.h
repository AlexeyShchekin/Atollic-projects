/*
 * DPS310.h
 *
 *  Created on: Dec 25, 2017
 *      Author: a.shchekin
 */

#ifndef DPS310_H_
#define DPS310_H_

#include "stm32l1xx_hal.h"
#include "Soft_I2C.h"

void ReadCoeffs(uint16_t device_addr, int * coeffs);
void CFG_Measurement(uint16_t device_addr, uint8_t PRECISION);

void GetReadings(uint16_t device_addr, int * coeffs, double * Readings);
int32_t GetTemperature(uint16_t device_addr);
int32_t GetPressure(uint16_t device_addr);
double CalibTemperature(int32_t Traw, int * coeffs);
double CalibPressure(int32_t Traw, int32_t Praw, int * coeffs);

#endif /* DPS310_H_ */
