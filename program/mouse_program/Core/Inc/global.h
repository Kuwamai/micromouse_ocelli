/*
 * global.h
 *
 *  Created on: Feb 14, 2023
 *      Author: kuwagata
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdint.h>
#include <mytypedef.h>

extern const float TIRE_DIAMETER;
extern const float M_PI;
extern const float VELOCITY_KP;
extern const float VELOCITY_KI;
extern const float VELOCITY_KD;
extern const float VELOCITY_MIN;
extern const float ANGULAR_VELOCITY_KP;
extern const float ANGULAR_VELOCITY_KI;
extern const float ANGULAR_VELOCITY_KD;
extern const float ANGULAR_VELOCITY_MIN;
extern const float WALL_CONTROL_KP;
extern const float WALL_CONTROL_KI;
extern const float WALL_CONTROL_KD;
extern const int DUTY_LIMIT;
extern const int STRAIGHT_MODE;
extern const int TURN_MODE;
extern const int LEFT;
extern const int RIGHT;
extern const float SENSOR_FL_TH;
extern const float SENSOR_L_TH;
extern const float SENSOR_R_TH;
extern const float SENSOR_FR_TH;
extern const float SENSOR_L_REF;
extern const float SENSOR_R_REF;

extern int sensor_count;
extern int battery_alert_count;
extern uint16_t adc_raw[5];
extern t_sensor sensor_fl;
extern t_sensor sensor_l;
extern t_sensor sensor_r;
extern t_sensor sensor_fr;
extern t_wall_control wall_control;

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
extern int turn_direction;
extern float angle_measured;
extern float angular_velocity_offset;
extern float angular_velocity;
extern float angular_velocity_ref;
extern float angular_velocity_ref_max;
extern float angular_velocity_err_past;
extern float angular_velocity_err_int;
extern float angular_accel;

extern const int MASK_SEARCH;
extern const float HALF_SECTION;    //半区画の距離
extern const float SECTION;        //一区画の距離
extern t_position mypos;
#define MAZESIZE_X 16 //迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAZESIZE_Y 16 //迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
extern t_wall			wall[MAZESIZE_X][MAZESIZE_Y];		//壁の情報を格納する構造体配列
extern unsigned char		map[MAZESIZE_X][MAZESIZE_Y];		//歩数マップ
#define UNKNOWN	2				//壁があるかないか判らない状態の場合の値
#define NOWALL	0				//壁がないばあいの値
#define WALL	1				//壁がある場合の値
#define VWALL	3				//仮想壁の値(未使用)
#define CONV_SEN2WALL(w) ((w) ? WALL : NOWALL)			//センサ情報から壁情報へ変換
#endif /* GLOBAL_H_ */
