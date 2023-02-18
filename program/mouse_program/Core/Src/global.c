/*
 * global.c
 *
 *  Created on: Feb 14, 2023
 *      Author: kuwagata
 */
#include <global.h>

// 車輪径 [m]
const float TIRE_DIAMETER = 0.0133;
// 円周率
const float M_PI = 3.141592;
// 速度制御Pゲイン
const float VELOCITY_KP = 800.0;
// 速度制御Iゲイン
const float VELOCITY_KI = 4.0;
// 速度制御Dゲイン
const float VELOCITY_KD = 500.0;
// 最低速度 [m/s]
const float VELOCITY_MIN = 0.1;
// 角速度制御Pゲイン
const float ANGULAR_VELOCITY_KP = 0.001;
// 角速度制御Iゲイン
const float ANGULAR_VELOCITY_KI = 0.0001;
// 角速度制御Dゲイン
const float ANGULAR_VELOCITY_KD = 0.0001;
// 最低角速度 [deg/s]
const float ANGULAR_VELOCITY_MIN = 30.0;
// 最大Duty比
const int DUTY_LIMIT = 200;
// 直進モード
const int STRAIGHT_MODE = 1;
// 旋回モード
const int TURN_MODE = 2;
// 左
const int LEFT = 1;
// 右
const int RIGHT = 2;
// 壁閾値 [V]
const float SENSOR_FL_TH = 0.1;
const float SENSOR_L_TH = 0.1;
const float SENSOR_R_TH = 0.1;
const float SENSOR_FR_TH = 0.1;
// 壁制御目標値 [V]
const float SENSOR_L_REF = 0.2;
const float SENSOR_R_REF = 0.2;

// センサ用割り込みカウンタ
int sensor_count = 0;
int battery_alert_count = 0;
// 12 bit AD値
uint16_t adc_raw[5] = {0};
t_sensor sensor_fl;
t_sensor sensor_l;
t_sensor sensor_r;
t_sensor sensor_fr;

// 12 bitエンコーダ値
int16_t encoder_l = 0;
// 12 bitエンコーダ値
int16_t encoder_r = 0;
// 過去の12 bitエンコーダ値
int16_t encoder_l_past = 0;
// 過去の12 bitエンコーダ値
int16_t encoder_r_past = 0;
// 12 bitエンコーダ値差分
int16_t encoder_l_diff = 0;
// 12 bitエンコーダ値差分
int16_t encoder_r_diff = 0;
// 左車輪速度 [m/s]
float velocity_l = 0;
// 右車輪速度 [m/s]
float velocity_r = 0;
// 左車輪目標速度 [m/s]
float velocity_l_ref = 0;
// 右車輪目標速度 [m/s]
float velocity_r_ref = 0;
// 両輪目標速度 [m/s]
float velocity_ref = 0;
// 最大両輪目標速度 [m/s]
float velocity_ref_max = 0;
// 過去の速度偏差 [m/s]
float velocity_l_err_past = 0;
// 過去の速度偏差 [m/s]
float velocity_r_err_past = 0;
// 積算偏差
float velocity_l_err_int = 0;
// 積算偏差
float velocity_r_err_int = 0;
// 入力加速度 [m/s/s]
float accel = 0;
// 走行距離 [m]
float length_run = 0;
// 走行モード
int run_mode = 0;
// 旋回方向
int turn_direction = 0;
// 機体角度(半時計周りが正) [deg]
float angle_measured = 0;
// 角速度オフセット [deg/s]
float angular_velocity_offset = 0;
// 実測角速度 [deg/s]
float angular_velocity = 0;
// 目標角速度 [deg/s]
float angular_velocity_ref = 0;
// 最大目標角速度 [deg/s]
float angular_velocity_ref_max = 0;
float angular_velocity_err_past = 0;
float angular_velocity_err_int = 0;
// 角加速度 [deg/s/s]
float angular_accel = 0;
