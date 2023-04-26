/*
 * run.c
 *
 *  Created on: Feb 16, 2023
 *      Author: kuwagata
 */

#include "main.h"
#include <global.h>
#include <delay_us.h>

extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi3;

void motor_on(void) {
  velocity_l_err_int = 0;
  velocity_r_err_int = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void motor_off(void) {
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

void imu_write1byte(uint8_t address, uint8_t data) {
  address = address & 0b01111111;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &address, 1, 1);
  HAL_SPI_Transmit(&hspi3, &data, 1, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
}

uint8_t imu_read1byte(uint8_t address) {
  uint8_t data = 0x00;
  address = address | 0b10000000;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, &address, 1, 1);
  HAL_SPI_Receive(&hspi3, &data, 1, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
  return data;
}

float read_angular_velocity(void) {
  uint8_t imu_spi_data_h = 0x00;
  uint8_t imu_spi_data_l = 0x00;
  int16_t imu_spi_data = 0x0000;

  imu_spi_data_h = imu_read1byte(0x37);
  delay_us(10);
  imu_spi_data_l = imu_read1byte(0x38);

  imu_spi_data = ((int16_t)((uint16_t)imu_spi_data_h<<8)) | ((int16_t)((uint16_t)imu_spi_data_l&0x00ff));
  float angular_velocity = (float)imu_spi_data / 65.5;
  return angular_velocity;
}

void imu_init(void) {
  imu_write1byte(0x06, 0x01);
  delay_us(10);

  imu_read1byte(0x00);
  HAL_Delay(50);

  float offset_sum = 0;
  for (int i = 0; i < 10; i++) {
    offset_sum += read_angular_velocity();
    HAL_Delay(5);
  }
  angular_velocity_offset = offset_sum / 10.0;
  angular_velocity_err_int = 0;
  angle_measured = 0;
}

void straight(float length, float accel_ref, float velocity_max, float velocity_end) {
  accel = accel_ref;
  run_mode = STRAIGHT_MODE;
  velocity_ref_max = velocity_max;
  motor_on();
  if (velocity_end == 0){
    while ((length - 0.01 - length_run) > (velocity_ref * velocity_ref) / (2.0 * accel_ref));
    accel = -accel_ref;
    while (length_run < length) {
      if (velocity_ref <= VELOCITY_MIN) {
        accel = 0;
        velocity_ref = VELOCITY_MIN;
      }
    }
    accel = 0;
    velocity_ref = 0;
    while (velocity_l >= 0 && velocity_r >= 0);
    motor_off();
  } else {
    while ((length - length_run) > ((velocity_ref * velocity_ref) - (velocity_end * velocity_end)) / (2.0 * accel_ref));
    accel = -accel_ref;
    while (length_run < length) {
      if (velocity_ref <= velocity_end) {
        accel = 0;
      }
    }
    accel = 0;
  }
  length_run = 0;
  run_mode = 0;
}

void turn(float turn_angle, float angular_accel_ref, float angular_velocity_max, short direction) {
  HAL_Delay(100);
  imu_init();
  angle_measured = 0;
  run_mode = TURN_MODE;
  motor_on();
  turn_direction = direction;
  if (direction == LEFT) {
    angular_accel = angular_accel_ref;
    angular_velocity_ref_max = angular_velocity_max;
    while ((turn_angle - angle_measured) > (angular_velocity_ref * angular_velocity_ref) / (2.0 * angular_accel_ref));
    angular_accel = -angular_accel_ref;
    while (angle_measured < turn_angle) {
      if (angular_velocity_ref <= ANGULAR_VELOCITY_MIN) {
        angular_accel = 0;
        angular_velocity_ref = ANGULAR_VELOCITY_MIN;
      }
    }
  } else if (direction == RIGHT) {
    angular_accel = -angular_accel_ref;
    angular_velocity_ref_max = -angular_velocity_max;
    while ((-turn_angle - angle_measured) < (angular_velocity_ref * angular_velocity_ref) / (2.0 * -angular_accel_ref));
    angular_accel = angular_accel_ref;
    while (angle_measured > -turn_angle) {
      if (angular_velocity_ref >= -ANGULAR_VELOCITY_MIN) {
        angular_accel = 0;
        angular_velocity_ref = -ANGULAR_VELOCITY_MIN;
      }
    }
  }
  angular_accel = 0;
  angular_velocity_ref = 0;
  while (angular_velocity >= 1.0 || angular_velocity <= -1.0);
  motor_off();
  turn_direction = 0;
  run_mode = 0;
  length_run = 0;
  HAL_Delay(100);
}


// ここからコピペ
void init_maze(void)	//迷路情報の初期化
{
	int i,j;
	
	for(i = 0; i < MAZESIZE_X; i++)
	{
		for(j = 0; j < MAZESIZE_Y; j++)
		{
			wall[i][j].north = wall[i][j].east = wall[i][j].south = wall[i][j].west = UNKNOWN;	//迷路の全体がわからない事を設定する
		}
	}
	
	for(i = 0; i < MAZESIZE_X; i++)
	{
		wall[i][0].south = WALL;		//四方の壁を追加する(南)
		wall[i][MAZESIZE_Y-1].north = WALL;	//四方の壁を追加する(北)
	}
	
	for(j = 0; j < MAZESIZE_Y; j++)
	{
		wall[0][j].west = WALL;			//四方の壁を追加する(西)
		wall[MAZESIZE_X-1][j].east = WALL;	//四方の壁を追加する(東)
	}
	
	wall[0][0].east = wall[1][0].west = WALL;	//スタート地点の右の壁を追加する
	
}

void init_map(int x, int y)
{
//迷路の歩数Mapを初期化する。全体を0xfe、引数の座標x,yは0で初期化する

  int i,j;
  
  for(i = 0; i < MAZESIZE_X; i++)		//迷路の大きさ分ループ(x座標)
  {
    for(j = 0; j < MAZESIZE_Y; j++)	//迷路の大きさ分ループ(y座標)
    {
      map[i][j] = 255;	//すべて255で埋める
    }
  }
  
  map[x][y] = 0;				//ゴール座標の歩数を０に設定
  
}

void make_map(int x, int y, int mask)	//歩数マップを作成する
{
//座標x,yをゴールとした歩数Mapを作成する。
//maskの値(MASK_SEARCH or MASK_SECOND)によって、
//探索用の歩数Mapを作るか、最短走行の歩数Mapを作るかが切り替わる

  int i,j;
  bool change_flag;			//Map作成終了を見極めるためのフラグ

  init_map(x,y);				//Mapを初期化する

  do
  {
    change_flag = false;				//変更がなかった場合にはループを抜ける
    for(i = 0; i < MAZESIZE_X; i++)			//迷路の大きさ分ループ(x座標)
    {
      for(j = 0; j < MAZESIZE_Y; j++)		//迷路の大きさ分ループ(y座標)
      {
        if(map[i][j] == 255)		//255の場合は次へ
        {
          continue;
        }
        
        if(j < MAZESIZE_Y-1)					//範囲チェック
        {
          if( (wall[i][j].north & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
          {
            if(map[i][j+1] == 255)			//まだ値が入っていなければ
            {
              map[i][j+1] = map[i][j] + 1;	//値を代入
              change_flag = true;		//値が更新されたことを示す
            }
          }
        }
      
        if(i < MAZESIZE_X-1)					//範囲チェック
        {
          if( (wall[i][j].east & mask) == NOWALL)		//壁がなければ
          {
            if(map[i+1][j] == 255)			//値が入っていなければ
            {
              map[i+1][j] = map[i][j] + 1;	//値を代入
              change_flag = true;		//値が更新されたことを示す
            }
          }
        }
      
        if(j > 0)						//範囲チェック
        {
          if( (wall[i][j].south & mask) == NOWALL)	//壁がなければ
          {
            if(map[i][j-1] == 255)			//値が入っていなければ
            {
              map[i][j-1] = map[i][j] + 1;	//値を代入
              change_flag = true;		//値が更新されたことを示す
            }
          }
        }
      
        if(i > 0)						//範囲チェック
        {
          if( (wall[i][j].west & mask) == NOWALL)		//壁がなければ
          {
            if(map[i-1][j] == 255)			//値が入っていなければ
            {
              map[i-1][j] = map[i][j] + 1;	//値を代入	
              change_flag = true;		//値が更新されたことを示す
            }
            
          }
          
        }
        
      }
      
    }
    
  }while(change_flag == true);	//全体を作り終わるまで待つ
  
}

void set_wall(int x, int y)	//壁情報を記録
{
//引数の座標x,yに壁情報を書き込む
  int n_write,s_write,e_write,w_write;
  
  
  //自分の方向に応じて書き込むデータを生成
  //CONV_SEN2WALL()はmacro.hを参照
  switch(mypos.dir){
    case north:	//北を向いている時
    
      n_write = CONV_SEN2WALL(sensor_fr.is_wall || sensor_fl.is_wall);	//前壁の有無を判断
      e_write = CONV_SEN2WALL(sensor_r.is_wall);				//右壁の有無を判断
      w_write = CONV_SEN2WALL(sensor_l.is_wall);				//左壁の有無を判断
      s_write = NOWALL;						//後ろは必ず壁がない
      
      break;
      
    case east:	//東を向いているとき
      
      e_write = CONV_SEN2WALL(sensor_fr.is_wall || sensor_fl.is_wall);	//前壁の有無を判断
      s_write = CONV_SEN2WALL(sensor_r.is_wall);				//右壁の有無を判断
      n_write = CONV_SEN2WALL(sensor_l.is_wall);				//左壁の有無を判断
      w_write = NOWALL;						//後ろは必ず壁がない
      
      break;
      
    case south:	//南を向いているとき
    
      s_write = CONV_SEN2WALL(sensor_fr.is_wall || sensor_fl.is_wall);	//前壁の有無を判断
      w_write = CONV_SEN2WALL(sensor_r.is_wall);				//右壁の有無を判断
      e_write = CONV_SEN2WALL(sensor_l.is_wall);				//左壁の有無を判断
      n_write = NOWALL;						//後ろは必ず壁がない

      break;
      
    case west:	//西を向いているとき
    
      w_write = CONV_SEN2WALL(sensor_fr.is_wall || sensor_fl.is_wall);	//前壁の有無を判断
      n_write = CONV_SEN2WALL(sensor_r.is_wall);				//右壁の有無を判断
      s_write = CONV_SEN2WALL(sensor_l.is_wall);				//左壁の有無を判断
      e_write = NOWALL;						//後ろは必ず壁がない
      
      break;
      
  }
  
  wall[x][y].north = n_write;	//実際に壁情報を書き込み
  wall[x][y].south = s_write;	//実際に壁情報を書き込み
  wall[x][y].east  = e_write;	//実際に壁情報を書き込み
  wall[x][y].west  = w_write;	//実際に壁情報を書き込み
  
  if(y < MAZESIZE_Y-1)	//範囲チェック
  {
    wall[x][y+1].south = n_write;	//反対側から見た壁を書き込み
  }
  
  if(x < MAZESIZE_X-1)	//範囲チェック
  {
    wall[x+1][y].west = e_write;	//反対側から見た壁を書き込み
  }
  
  if(y > 0)	//範囲チェック
  {
    wall[x][y-1].north = s_write;	//反対側から見た壁を書き込み
  }
  
  if(x > 0)	//範囲チェック
  {
    wall[x-1][y].east = w_write;	//反対側から見た壁を書き込み
  }
  
}

bool is_unknown(int x, int y)	//指定された区画が未探索か否かを判断する関数 未探索:true　探索済:false
{
  //座標x,yが未探索区間か否かを調べる
  
  if((wall[x][y].north == UNKNOWN) || (wall[x][y].east == UNKNOWN) || (wall[x][y].south == UNKNOWN) || (wall[x][y].west == UNKNOWN))
  {			//どこかの壁情報が不明のままであれば
    return true;	//未探索
  }
  else
  {
    return false;	//探索済
  }
}

int get_priority(int x, int y, t_direction dir)	//そのマスの情報から、優先度を算出する
{
  //座標x,yと、向いている方角dirから優先度を算出する
  
  //未探索が一番優先度が高い.(4)
  //それに加え、自分の向きと、行きたい方向から、
  //前(2)横(1)後(0)の優先度を付加する。

  int priority;	//優先度を記録する変数
  
  priority = 0;
  
  if(mypos.dir == dir)				//行きたい方向が現在の進行方向と同じ場合
  {
    priority = 2;
  }
  else if( ((4+mypos.dir-dir)%4) == 2)		//行きたい方向が現在の進行方向と逆の場合
  {
    priority = 0;
  }
  else						//それ以外(左右どちらか)の場合
  {
    priority = 1;
  }
  
  
  if(is_unknown(x,y) == true)
  {
    priority += 4;				//未探索の場合優先度をさらに付加
  }
  
  return priority;				//優先度を返す
  
}

int get_nextdir(int x, int y, int mask, t_direction *dir)	
{
  //ゴール座標x,yに向かう場合、今どちらに行くべきかを判断する。
  //探索、最短の切り替えのためのmaskを指定、dirは方角を示す
  int little,priority,tmp_priority;		//最小の値を探すために使用する変数


  make_map(x,y,mask);				//歩数Map生成
  little = 255;					//最小歩数を255歩(mapがunsigned char型なので)に設定	

  priority = 0;					//優先度の初期値は0
  
    //maskの意味はstatic_parameter.hを参照
  if( (wall[mypos.x][mypos.y].north & mask) == NOWALL)			//北に壁がなければ
  {
    tmp_priority = get_priority(mypos.x, mypos.y + 1, north);	//優先度を算出
    if(map[mypos.x][mypos.y+1] < little)				//一番歩数が小さい方向を見つける
    {
      little = map[mypos.x][mypos.y+1];			//ひとまず北が歩数が小さい事にする
      *dir = north;						//方向を保存
      priority = tmp_priority;				//優先度を保存
    }
    else if(map[mypos.x][mypos.y+1] == little)			//歩数が同じ場合は優先度から判断する
    {
      if(priority < tmp_priority )				//優先度を評価
      {
        *dir = north;					//方向を更新
        priority = tmp_priority;			//優先度を保存
      }
    }
  }
  
  if( (wall[mypos.x][mypos.y].east & mask) == NOWALL)			//東に壁がなければ
  {
    tmp_priority = get_priority(mypos.x + 1, mypos.y, east);	//優先度を算出
    if(map[mypos.x + 1][mypos.y] < little)				//一番歩数が小さい方向を見つける
    {
      little = map[mypos.x+1][mypos.y];			//ひとまず東が歩数が小さい事にする
      *dir = east;						//方向を保存
      priority = tmp_priority;				//優先度を保存
    }
    else if(map[mypos.x + 1][mypos.y] == little)			//歩数が同じ場合、優先度から判断
    {
      if(priority < tmp_priority)				//優先度を評価
      {
        *dir = east;					//方向を保存
        priority = tmp_priority;			//優先度を保存
      }
    }
  }
  
  if( (wall[mypos.x][mypos.y].south & mask) == NOWALL)			//南に壁がなければ
  {
    tmp_priority = get_priority(mypos.x, mypos.y - 1, south);	//優先度を算出
    if(map[mypos.x][mypos.y - 1] < little)				//一番歩数が小さい方向を見つける
    {
      little = map[mypos.x][mypos.y-1];			//ひとまず南が歩数が小さい事にする
      *dir = south;						//方向を保存
      priority = tmp_priority;				//優先度を保存
    }
    else if(map[mypos.x][mypos.y - 1] == little)			//歩数が同じ場合、優先度で評価
    {
      if(priority < tmp_priority)				//優先度を評価
      {
        *dir = south;					//方向を保存
        priority = tmp_priority;			//優先度を保存
      }
    }
  }
  
  if( (wall[mypos.x][mypos.y].west & mask) == NOWALL)			//西に壁がなければ
  {
    tmp_priority = get_priority(mypos.x - 1, mypos.y, west);	//優先度を算出
    if(map[mypos.x-1][mypos.y] < little)				//一番歩数が小さい方向を見つける
    {
      little = map[mypos.x-1][mypos.y];			//西が歩数が小さい
      *dir = west;						//方向を保存
      priority = tmp_priority;				//優先度を保存
    }
    else if(map[mypos.x - 1][mypos.y] == little)			//歩数が同じ場合、優先度で評価
    {
      *dir = west;						//方向を保存
      priority = tmp_priority;				//優先度を保存
    }
  }


  return ( (int)( ( 4 + *dir - mypos.dir) % 4 ) );			//どっちに向かうべきかを返す。
                    //演算の意味はmytyedef.h内のenum宣言から。
  
}

void search_adachi(int gx, int gy, float search_velocity, float search_accel, float turn_accel, float turn_speed)
{
//引数gx,gyに向かって足立法で迷路を探索する
  t_direction glob_nextdir;					//次に向かう方向を記録する変数

  accel=search_accel;
  length_run = 0;

  switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))		//次に行く方向を戻り値とする関数を呼ぶ
  {
    case front:
      
      straight(HALF_SECTION,search_accel,search_velocity,search_velocity);		//半区画進む
      break;
    
    case right:
      turn(90,turn_accel,turn_speed,RIGHT);				//右に曲がって
      straight(HALF_SECTION,search_accel,search_velocity,search_velocity);		//半区画進む
      break;
    
    case left:
      turn(90,turn_accel,turn_speed,LEFT);				//左に曲がって
      straight(HALF_SECTION,search_accel,search_velocity,search_velocity);		//半区画進む
      break;
    
    case rear:
      turn(180,turn_accel,turn_speed,RIGHT);					//180ターン
      straight(HALF_SECTION,search_accel,search_velocity,search_velocity);		//半区画進む
      break;
  }
    accel=search_accel;				//加速度を設定
    //con_wall.enable = true;					//壁制御を有効にする
    //wall_control.enable = true;    // いらないかも
    //MOT_CWCCW_R = MOT_CWCCW_L = MOT_FORWARD;		//前方に進む
    //len_mouse = 0;					//進んだ距離カウント用変数をリセット
    //MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1;		//カウントスタート
  
    mypos.dir = glob_nextdir;				//方向を更新

  //向いた方向によって自分の座標を更新する
  switch(mypos.dir)
  {
    case north:
      mypos.y++;	//北を向いた時はY座標を増やす
      break;
      
    case east:
      mypos.x++;	//東を向いた時はX座標を増やす
      break;
      
    case south:
      mypos.y--;	//南を向いた時はY座標を減らす
      break;
    
    case west:
      mypos.x--;	//西を向いたときはX座標を減らす
      break;

  }

  
  while((mypos.x != gx) || (mypos.y != gy)){				//ゴールするまで繰り返す

    set_wall(mypos.x,mypos.y);					//壁をセット

    switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))		//次に行く方向を戻り値とする関数を呼ぶ
    {
      case front:
        led_control(0b0001);
        straight(SECTION,search_accel,search_velocity,search_velocity);		//半区画進む
        break;
    
      case right:
        led_control(0b0010);
        straight(HALF_SECTION,search_accel,search_velocity,0);		//半区画進む
        turn(90,turn_accel,turn_speed,RIGHT);				//右に曲がって
        straight(HALF_SECTION,search_accel,search_velocity,search_velocity);
        break;
      
      case left:
        led_control(0b0100);
        straight(HALF_SECTION,search_accel,search_velocity,0);		//半区画進む
        turn(90,turn_accel,turn_speed,LEFT);				//左に曲がって
        straight(HALF_SECTION,search_accel,search_velocity,search_velocity);
        break;
      
      case rear:
        led_control(0b1000);
        straight(HALF_SECTION,search_accel,search_velocity,0);		//半区画進む
        turn(180,turn_accel,turn_speed,RIGHT);					//180ターン
        straight(HALF_SECTION,search_accel,search_velocity,search_velocity);
        break;
    }

    //con_wall.enable = true;						//壁制御を有効にする
    //len_mouse = 0;						//進んだ距離をカウントする変数をリセット
    //MTU.TSTR.BIT.CST3 = MTU.TSTR.BIT.CST4 = 1;			//カウントスタート
  
    mypos.dir = glob_nextdir;					//方向を更新
    
    //向いた方向によって自分の座標を更新する
    switch(mypos.dir)
    {
      case north:
        mypos.y++;	//北を向いた時はY座標を増やす
        break;
        
      case east:
        mypos.x++;	//東を向いた時はX座標を増やす
        break;
        
      case south:
        mypos.y--;	//南を向いた時はY座標を減らす
        break;
      
      case west:
        mypos.x--;	//西を向いたときはX座標を減らす
        break;

    }
    
  }
  set_wall(mypos.x,mypos.y);		//壁をセット
  straight(HALF_SECTION,search_accel,search_velocity,0);	
}
