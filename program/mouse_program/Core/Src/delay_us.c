/*
 * delay_us.c
 *
 *  Created on: Feb 3, 2023
 *      Author: kuwagata
 */

#include <stm32f4xx_hal.h>
void delay_us(TIM_HandleTypeDef *htim, uint16_t us) {
  __HAL_TIM_SET_COUNTER(htim, 0);
  while(__HAL_TIM_GET_COUNTER(htim) < us);
  return;
}
