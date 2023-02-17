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
extern const float VELOCITY_KP;
extern const float VELOCITY_KI;
extern const float VELOCITY_KD;
extern const float VELOCITY_MIN;
extern const int DUTY_LIMIT;
extern const int STRAIGHT_MODE;
extern const int TURN_MODE;

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
extern float velocity_l_ref;
extern float velocity_r_ref;
extern float velocity_ref;
extern float velocity_ref_max;
extern float velocity_l_err_past;
extern float velocity_r_err_past;
extern float velocity_l_err_int;
extern float velocity_r_err_int;
extern float accel;
extern float length_run;
extern int run_mode;
extern float angle_measured;
extern float angular_velocity_offset;
#endif /* GLOBAL_H_ */
