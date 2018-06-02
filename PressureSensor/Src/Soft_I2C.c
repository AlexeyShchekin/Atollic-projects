/*
 * Soft_I2C.c
 *
 *  Created on: Dec 26, 2017
 *      Author: a.shchekin
 */

#include "Soft_I2C.h"

static void Delay(void)
{
    volatile uint16_t i = 10;	//	100kHz on 32MHz
    while ( i ) {
        i--;
    }
}

uint8_t i2cSoft_Start()
{
    SDAH;
    SCLH;
    Delay();
    if ( !(SDAread) )
        return 0x00;
    SDAL;
    Delay();
    if ( SDAread )
        return 0x00;
    Delay();
    return 0x01;
}

void i2cSoft_Stop()
{
    SCLL;                       // Stop sequence
    Delay();
    SDAL;
    Delay();
    SCLH;
    Delay();
    SDAH;
    Delay();
}

void i2cSoft_Ack (void)
{
    SCLL;
    Delay();
    SDAL;
    Delay();
    SCLH;
    Delay();
    SCLL;
    Delay();
}

void i2cSoft_NoAck (void)
{
    SCLL;
    Delay();
    SDAH;
    Delay();
    SCLH;
    Delay();
    SCLL;
    Delay();
}

uint8_t i2cSoft_WaitAck(void)
{
    SCLL;
    Delay();
    SDAH;
    Delay();
    SCLH;
    Delay();
    if ( SDAread ) {
        SCLL;
        return 0x00;
    }
    SCLL;
    return 0x01;
}

void i2cSoft_PutByte ( uint8_t data )
{
    uint8_t i = 8;
    while ( i-- ) {
        SCLL;
        Delay();
        if ( data & 0x80 )
            SDAH;
        else
            SDAL;
        data <<= 1;
        Delay();
        SCLH;
        Delay();
    }
    SCLL;
}

uint8_t i2cSoft_GetByte (void)
{
    volatile uint8_t i = 8;
    uint8_t data = 0;

    SDAH;
    while ( i-- ) {
        data <<= 1;
        SCLL;
        Delay();
        SCLH;
        Delay();
        if ( SDAread ) {
            data |= 0x01;
        }
    }
    SCLL;
    return data;
}

int i2cSoft_WriteBuffer ( uint8_t chipAddress, uint8_t *buffer, uint32_t sizeOfBuffer )
{
    if ( !i2cSoft_Start() )
        return I2C_RESULT_ERROR;

    i2cSoft_PutByte( chipAddress );
    if ( !i2cSoft_WaitAck() ) {
       // i2cSoft_Stop();
        //return I2C_RESULT_ERROR;
    }

    while ( sizeOfBuffer != 0 ) {
        i2cSoft_PutByte( *buffer );
        if ( !i2cSoft_WaitAck() ) {
            i2cSoft_Stop();
            return I2C_RESULT_ERROR;
        }

        buffer++;
        sizeOfBuffer--;
    }
    i2cSoft_Stop();
    return I2C_RESULT_SUCCESS;
}

int i2cSoft_WriteCMD ( uint8_t chipAddress, uint8_t CMD )
{
    if ( !i2cSoft_Start() )
        return I2C_RESULT_ERROR;

    i2cSoft_PutByte( chipAddress );
    if ( !i2cSoft_WaitAck() ) {
       // i2cSoft_Stop();
        //return I2C_RESULT_ERROR;
    }

	i2cSoft_PutByte( CMD );
	if ( !i2cSoft_WaitAck() ) {
		//i2cSoft_Stop();
		//return I2C_RESULT_ERROR;
	}

    //i2cSoft_Stop();
    return I2C_RESULT_SUCCESS;
}

int i2cSoft_ReadBuffer ( uint8_t chipAddress, uint8_t *buffer, uint32_t sizeOfBuffer )
{
    if ( !i2cSoft_Start() )
        return I2C_RESULT_ERROR;

    i2cSoft_PutByte( chipAddress + 1 );
    if ( !i2cSoft_WaitAck() ) {
       // i2cSoft_Stop();
       // return I2C_RESULT_ERROR;
    }

    while ( sizeOfBuffer != 0 ) {
        *buffer = i2cSoft_GetByte();

        buffer++;
        sizeOfBuffer--;
        if ( sizeOfBuffer == 0 ) {
            i2cSoft_NoAck();
            break;
        }
        else
            i2cSoft_Ack();
    }
    i2cSoft_Stop();
    return I2C_RESULT_SUCCESS;
}
