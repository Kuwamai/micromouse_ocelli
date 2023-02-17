/*
 * run.c
 *
 *  Created on: Feb 16, 2023
 *      Author: kuwagata
 */

#include <global.h>
#include "main.h"

extern TIM_HandleTypeDef htim1;

void motor_on(void) {
  velocity_l_err_int = 0;
  velocity_r_err_int = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void motor_off(void) {
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

void straight(float length, float velocity_max, float accel_ref) {
  length_run = 0;
  accel = accel_ref;
  run_mode = STRAIGHT_MODE;
  velocity_ref_max = velocity_max;
  motor_on();
  while ((length - length_run) > (velocity_ref * velocity_ref) / (2.0 * accel_ref));
  accel = -accel_ref;
  while (length_run < length) {
    if (velocity_ref <= VELOCITY_MIN) {
      accel = 0;
      velocity_ref = VELOCITY_MIN;
    }
  }
  accel = 0;
  velocity_ref = 0;
  while (velocity_l >= 0 && velocity_r >= 0);
  motor_off();
  run_mode = 0;
}
