/*
 * global.h
 *
 *  Created on: Feb 14, 2023
 *      Author: kuwagata
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdint.h>

extern int sensor_count;
extern uint16_t adc_raw[5];
extern float adc_voltage[5];
extern int16_t encoder_l;
extern int16_t encoder_r;
#endif /* GLOBAL_H_ */
