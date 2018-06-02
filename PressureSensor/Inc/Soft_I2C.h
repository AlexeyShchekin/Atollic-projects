/*
 * Soft_I2C.h
 *
 *  Created on: Dec 26, 2017
 *      Author: a.shchekin
 */

#ifndef SOFT_I2C_H_
#define SOFT_I2C_H_

#include "stm32l1xx_hal.h"

#define SCLH                        (GPIOB->BSRR |= (uint32_t)GPIO_PIN_6)
#define SCLL                        (GPIOB->BSRR |= (uint32_t)GPIO_PIN_6 << 16)

#define SDAH                        (GPIOB->BSRR |= (uint32_t)GPIO_PIN_7)
#define SDAL                        (GPIOB->BSRR |= (uint32_t)GPIO_PIN_7 << 16)
#define SCLread                     ((GPIOB->IDR & GPIO_PIN_6)==GPIO_PIN_6)
#define SDAread                     ((GPIOB->IDR & GPIO_PIN_7)==GPIO_PIN_7)

#define I2C_RESULT_SUCCESS          0
#define I2C_RESULT_ERROR            (-1)

uint8_t i2cSoft_Start();
void i2cSoft_Stop();
void i2cSoft_Ack (void);
void i2cSoft_NoAck (void);
uint8_t i2cSoft_WaitAck(void);
void i2cSoft_PutByte ( uint8_t data );
uint8_t i2cSoft_GetByte (void);
int i2cSoft_WriteBuffer ( uint8_t chipAddress, uint8_t *buffer, uint32_t sizeOfBuffer );
int i2cSoft_WriteCMD ( uint8_t chipAddress, uint8_t CMD );
int i2cSoft_ReadBuffer ( uint8_t chipAddress, uint8_t *buffer, uint32_t sizeOfBuffer );

#endif /* SOFT_I2C_H_ */
