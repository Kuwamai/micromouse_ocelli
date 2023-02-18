/*
 * mytypedef.h
 *
 *  Created on: Feb 18, 2023
 *      Author: kuwagata
 */

#ifndef INC_MYTYPEDEF_H_
#define INC_MYTYPEDEF_H_

#include <stdbool.h>

typedef struct
{
  float value;   // センサ値 [V]
  float error;   // 壁制御偏差 [V]
  bool is_wall;  // 壁の有無
}t_sensor;

typedef struct
{
  float error;  // 壁制御偏差
  float error_past;   // 過去の壁制御偏差 [V]
  float error_int;   // 壁制御偏差積算値 [V]
  float error_diff;   // 壁制御偏差差分 [V]
}t_wall_control;



#endif /* INC_MYTYPEDEF_H_ */
