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
void imu_write1byte(uint8_t address, uint8_t data);
uint8_t imu_read1byte(uint8_t address);
float read_angular_velocity(void);
void imu_init(void);
void straight(float length, float accel_ref, float velocity_max, float velocity_end);
void turn(float turn_angle, float angular_accel_ref, float angular_velocity_max, short direction);
void search_adachi(int gx, int gy, float search_velocity, float search_accel, float turn_accel, float turn_speed);
#endif /* INC_RUN_H_ */
