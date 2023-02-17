/*
 * run.h
 *
 *  Created on: Feb 16, 2023
 *      Author: kuwagata
 */

#ifndef INC_RUN_H_
#define INC_RUN_H_

void motor_on(void);
void motor_off(void);
void straight(float length, float velocity_max, float accel_ref);
#endif /* INC_RUN_H_ */
