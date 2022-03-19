#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "chassis_task.h"

#define K_NORMAL_SPEED					10.0
#define	K_LOW_SPEED							5.0
#define	K_FULL_SPEED						15.0

typedef enum
{
  CHASSIS_VECTOR_ZERO_FORCE,            //底盘无力, 跟没上电那样
  CHASSIS_STOP,   											//底盘强制停止
  CHASSIS_NORMAL_SPEED,                	//常速模式
	CHASSIS_LOW_SPEED,										//低速模式
	CHASSIS_HIGH_SPEED										//高速模式
} chassis_behaviour_e;

/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif

