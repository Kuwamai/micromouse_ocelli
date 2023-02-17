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

void led_control(uint8_t led_state) {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, led_state & 0x01);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, led_state & 0x02);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, led_state & 0x04);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, led_state & 0x08);
}