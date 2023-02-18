/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <global.h>
#include <delay_us.h>
#include <run.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  // センサ値の取得
  uint16_t pins[4] = {GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_0, GPIO_PIN_1};
  GPIO_TypeDef* ports[4] = {GPIOB, GPIOB, GPIOA, GPIOA};
  const float adjust_volt = 1.011;
  float battery_voltage = 0;

  switch(sensor_count) {
    case 0:
      HAL_GPIO_WritePin(ports[sensor_count], pins[sensor_count], GPIO_PIN_SET);
      delay_us(15);
      sensor_fl.value = (float)adc_raw[sensor_count] * 3.3 / 4096.0;
      HAL_GPIO_WritePin(ports[sensor_count], pins[sensor_count], GPIO_PIN_RESET);
      sensor_count++;
      break;
    case 1:
      HAL_GPIO_WritePin(ports[sensor_count], pins[sensor_count], GPIO_PIN_SET);
      delay_us(15);
      sensor_l.value = (float)adc_raw[sensor_count] * 3.3 / 4096.0;
      HAL_GPIO_WritePin(ports[sensor_count], pins[sensor_count], GPIO_PIN_RESET);
      sensor_count++;
      break;
    case 2:
      HAL_GPIO_WritePin(ports[sensor_count], pins[sensor_count], GPIO_PIN_SET);
      delay_us(15);
      sensor_r.value = (float)adc_raw[sensor_count] * 3.3 / 4096.0;
      HAL_GPIO_WritePin(ports[sensor_count], pins[sensor_count], GPIO_PIN_RESET);
      sensor_count++;
      break;
    case 3:
      HAL_GPIO_WritePin(ports[sensor_count], pins[sensor_count], GPIO_PIN_SET);
      delay_us(15);
      sensor_fr.value = (float)adc_raw[sensor_count] * 3.3 / 4096.0;
      HAL_GPIO_WritePin(ports[sensor_count], pins[sensor_count], GPIO_PIN_RESET);
      sensor_count++;
      break;
    case 4:
      battery_voltage = (float)adc_raw[sensor_count] * 3.3 / 4096.0;
      battery_voltage = battery_voltage / 2.0 * 3.0 * adjust_volt;
      if (battery_voltage < 3.6) {
        battery_alert_count++;
        if (battery_alert_count > 100) led_control(0xF);
        if (battery_alert_count > 200) {
          led_control(0);
          battery_alert_count = 0;
        }
        if (battery_voltage < 3.5) {
          led_control(0xF);
          motor_off();
          while(1);
        }
      }
      sensor_count++;
      break;
    default:
      sensor_count = 0;
      break;
  }

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
  if (run_mode == STRAIGHT_MODE) {
    velocity_ref += accel / 1000.0;
    if (velocity_ref > velocity_ref_max) {
      velocity_ref = velocity_ref_max;
    }
    velocity_l_ref = velocity_ref;
    velocity_r_ref = velocity_ref;
  }
  // 角速度制御
  if (run_mode == TURN_MODE) {
    angular_velocity_ref += angular_accel / 1000.0;
    if (turn_direction == LEFT) {
      if (angular_velocity_ref > angular_velocity_ref_max) {
        angular_velocity_ref = angular_velocity_ref_max;
      }
    } else if (turn_direction == RIGHT) {
      if (angular_velocity_ref < angular_velocity_ref_max) {
        angular_velocity_ref = angular_velocity_ref_max;
      }
    }
    angular_velocity = read_angular_velocity() - angular_velocity_offset;
    angle_measured += angular_velocity / 1000.0;

    float angular_velocity_err = angular_velocity_ref - angular_velocity;
    angular_velocity_err_int = angular_velocity_err_int + angular_velocity_err;
    if (angular_velocity_err_int >  1000) angular_velocity_err_int =  1000;
    if (angular_velocity_err_int < -1000) angular_velocity_err_int = -1000;
    float angular_velocity_err_diff = angular_velocity_err_past - angular_velocity_err;
    angular_velocity_err_past = angular_velocity_err;
    // 目標速度差 [m/s]
    float velocity_ref_diff = angular_velocity_err * ANGULAR_VELOCITY_KP + angular_velocity_err_int * ANGULAR_VELOCITY_KI + angular_velocity_err_diff * ANGULAR_VELOCITY_KD;
    velocity_l_ref = -velocity_ref_diff;
    velocity_r_ref =  velocity_ref_diff;
  }

  // 速度制御
  uint16_t encoder_address = 0xFFFF;
  uint16_t encoder_spi_data;
  int16_t encoder_l_diff_raw = 0;
  int16_t encoder_r_diff_raw = 0;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&encoder_address, (uint8_t*)&encoder_spi_data, 1, 1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  encoder_l = (encoder_spi_data & 0b0011111111111100) >> 2;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&encoder_address, (uint8_t*)&encoder_spi_data, 1, 1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  encoder_r = (encoder_spi_data & 0b0011111111111100) >> 2;

  encoder_l_diff_raw = encoder_l_past - encoder_l;
  if (encoder_l_diff_raw < -2000) {
    encoder_l_diff = encoder_l_diff_raw + 4095;
  } else if (encoder_l_diff_raw > 2000) {
    encoder_l_diff = encoder_l_diff_raw - 4095;
  } else {
    encoder_l_diff = encoder_l_diff_raw;
  }

  encoder_r_diff_raw = encoder_r_past - encoder_r;
  if (encoder_r_diff_raw < -2000) {
    encoder_r_diff = encoder_r_diff_raw + 4095;
  } else if (encoder_r_diff_raw > 2000) {
    encoder_r_diff = encoder_r_diff_raw - 4095;
  } else {
    encoder_r_diff = encoder_r_diff_raw;
  }

  encoder_l_past = encoder_l;
  encoder_r_past = encoder_r;

  velocity_l = -((float)encoder_l_diff / 4095.0) * TIRE_DIAMETER * M_PI * 1000.0;
  velocity_r =  ((float)encoder_r_diff / 4095.0) * TIRE_DIAMETER * M_PI * 1000.0;

  length_run += (velocity_l + velocity_r) / 2.0 / 1000.0;

  float velocity_l_err = velocity_l_ref - velocity_l;
  velocity_l_err_int = velocity_l_err_int + velocity_l_err;
  if (velocity_l_err_int >  100) velocity_l_err_int =  100;
  if (velocity_l_err_int < -100) velocity_l_err_int = -100;
  float velocity_l_err_diff = velocity_l_err_past - velocity_l_err;
  velocity_l_err_past = velocity_l_err;
  int duty_l = (int)(velocity_l_err * VELOCITY_KP + velocity_l_err_int * VELOCITY_KI + velocity_l_err_diff * VELOCITY_KD);

  float velocity_r_err = velocity_r_ref - velocity_r;
  velocity_r_err_int = velocity_r_err_int + velocity_r_err;
  if (velocity_r_err_int >  100) velocity_r_err_int =  100;
  if (velocity_r_err_int < -100) velocity_r_err_int = -100;
  float velocity_r_err_diff = velocity_r_err_past - velocity_r_err;
  velocity_r_err_past = velocity_r_err;
  int duty_r = (int)(velocity_r_err * VELOCITY_KP + velocity_r_err_int * VELOCITY_KI + velocity_r_err_diff * VELOCITY_KD);

  if (duty_l >= 0) {
    if (duty_l > DUTY_LIMIT) duty_l = DUTY_LIMIT;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_l);
  } else {
    duty_l = -duty_l;
    if (duty_l > DUTY_LIMIT) duty_l = DUTY_LIMIT;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_l);
  }

  if (duty_r >= 0) {
    if (duty_r > DUTY_LIMIT) duty_r = DUTY_LIMIT;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty_r);
  } else {
    duty_r = -duty_r;
    if (duty_r > DUTY_LIMIT) duty_r = DUTY_LIMIT;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty_r);
  }
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
