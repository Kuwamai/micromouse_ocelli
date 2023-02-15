/*
 * global.c
 *
 *  Created on: Feb 14, 2023
 *      Author: kuwagata
 */
#include <global.h>

int sensor_count = 0;
uint16_t adc_raw[5];
float adc_voltage[5];
int16_t encoder_l;
int16_t encoder_r;
int16_t encoder_l_past;
int16_t encoder_r_past;
int16_t encoder_l_diff;
int16_t encoder_r_diff;