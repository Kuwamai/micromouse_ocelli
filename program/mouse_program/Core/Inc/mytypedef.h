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
  bool enable;
}t_wall_control;

// ここから先コピペ
typedef enum
{
	north=0,
	east=1,
	south=2,
	west=3,
}t_direction;

typedef enum
{
	front=0,		//前
	right=1,		//右
	rear=2,			//後
	left=3,			//左
	unknown,		//方向不明
}t_local_dir;	//自分から見た方向を示す列挙型

typedef struct
{
	short x;
	short y;
	t_direction dir;
}t_position;

typedef struct
{
	unsigned char north:2;	//北の壁情報
	unsigned char east:2;	//東の壁情報
	unsigned char south:2;	//南の壁情報
	unsigned char west:2;	//西の壁情報
}t_wall;			//壁情報を格納する構造体(ビットフィールド)

#endif /* INC_MYTYPEDEF_H_ */
