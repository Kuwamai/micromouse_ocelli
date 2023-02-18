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
  bool is_wall;  // 壁の有無
}t_sensor;



#endif /* INC_MYTYPEDEF_H_ */
