/*
 * DWT_Delay.h
 *
 *  Created on: Dec 26, 2017
 *      Author: a.shchekin
 */

#ifndef DWT_DELAY_H_
#define DWT_DELAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l1xx_hal.h"

uint32_t DWT_Delay_Init(void);

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000L);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

#ifdef __cplusplus
}
#endif

#endif /* DWT_DELAY_H_ */
