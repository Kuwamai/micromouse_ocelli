/*
 * global.h
 *
 *  Created on: Feb 14, 2023
 *      Author: kuwagata
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdint.h>

extern const float TIRE_DIAMETER;
extern const float M_PI;

extern int sensor_count;
extern uint16_t adc_raw[5];
extern float adc_voltage[5];

extern int16_t encoder_l;
extern int16_t encoder_r;
extern int16_t encoder_l_past;
extern int16_t encoder_r_past;
extern int16_t encoder_l_diff;
extern int16_t encoder_r_diff;
extern float velocity_l;
extern float velocity_r;
extern float velocity_ref;
#endif /* GLOBAL_H_ */
