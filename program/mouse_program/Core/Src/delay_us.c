/*
 * delay_us.c
 *
 *  Created on: Feb 3, 2023
 *      Author: kuwagata
 */

#include <stm32f4xx_hal.h>
void delay_us(uint16_t us) {
  extern TIM_HandleTypeDef htim2;
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while(__HAL_TIM_GET_COUNTER(&htim2) < us);
  return;
}

void led_control(uint8_t led_state) {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, led_state & 0x01);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, led_state & 0x02);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, led_state & 0x04);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, led_state & 0x08);
}