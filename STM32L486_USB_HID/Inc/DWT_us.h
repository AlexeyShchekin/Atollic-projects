/*
 * DWT_us.h
 *
 *  Created on: 1 рту. 2018 у.
 *      Author: Alexey
 */

#ifndef DWT_US_H_
#define DWT_US_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

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

#endif /* DWT_US_H_ */
