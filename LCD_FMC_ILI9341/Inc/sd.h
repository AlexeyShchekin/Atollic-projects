/*
 * sd.h
 *
 *  Created on: 23 ���. 2019 �.
 *      Author: Alexey
 */

#ifndef SD_H_
#define SD_H_

#include "stm32f7xx_hal.h"

#define CS_SD_GPIO_PORT GPIOA
#define CS_SD_PIN GPIO_PIN_3
#define SS_SD_SELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_RESET)
#define SS_SD_DESELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_SET)
#define LD_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //Blue
#define LD_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //Blue

 /* Card type flags (CardType) */
#define CT_MMC 0x01 /* MMC ver 3 */
#define CT_SD1 0x02 /* SD ver 1 */
#define CT_SD2 0x04 /* SD ver 2 */
#define CT_SDC (CT_SD1|CT_SD2) /* SD */
#define CT_BLOCK 0x08 /* Block addressing */

typedef struct sd_info {
	volatile uint8_t type;
} sd_info_ptr;

void SD_PowerOn(void);
uint8_t sd_ini(void);
void SPI_SendByte(uint8_t bt);
uint8_t SPI_ReceiveByte(void);
void SPI_Release(void);

#endif /* SD_H_ */