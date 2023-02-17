/*
 * run.c
 *
 *  Created on: Feb 16, 2023
 *      Author: kuwagata
 */

#include "main.h"
#include <global.h>
#include <delay_us.h>

extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi3;

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

void imu_write1byte(uint8_t address, uint8_t data) {
  address = address & 0b01111111;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &address, 1, 1);
  HAL_SPI_Transmit(&hspi3, &data, 1, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
}

uint8_t imu_read1byte(uint8_t address) {
  uint8_t data = 0x00;
  address = address | 0b10000000;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &address, 1, 1);
  HAL_SPI_Receive(&hspi3, &data, 1, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
  return data;
}

float read_angular_velocity(void) {
  uint8_t imu_spi_data_h = 0x00;
  uint8_t imu_spi_data_l = 0x00;
  int16_t imu_spi_data = 0x0000;

  imu_spi_data_h = imu_read1byte(0x37);
  delay_us(10);
  imu_spi_data_l = imu_read1byte(0x38);

  imu_spi_data = ((int16_t)((uint16_t)imu_spi_data_h<<8)) | ((int16_t)((uint16_t)imu_spi_data_l&0x00ff));
  float angular_velocity = (float)imu_spi_data / 65.5;
  return angular_velocity;
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

/*
void turn(float turn_degree, float angular_accel, float angular_velocity_max, short dir) {

}*/