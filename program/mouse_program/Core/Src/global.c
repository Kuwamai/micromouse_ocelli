/*
 * global.c
 *
 *  Created on: Feb 14, 2023
 *      Author: kuwagata
 */
#include <global.h>

const float TIRE_DIAMETER = 0.013;
const float M_PI = 3.141592;

int sensor_count = 0;
uint16_t adc_raw[5] = {0};
float adc_voltage[5] = {0};

int16_t encoder_l = 0;
int16_t encoder_r = 0;
int16_t encoder_l_past = 0;
int16_t encoder_r_past = 0;
int16_t encoder_l_diff = 0;
int16_t encoder_r_diff = 0;
float velocity_l = 0;
float velocity_r = 0;
float velocity_ref = 0;