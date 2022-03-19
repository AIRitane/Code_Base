#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "chassis_task.h"

#define K_NORMAL_SPEED					10.0
#define	K_LOW_SPEED							5.0
#define	K_FULL_SPEED						15.0

typedef enum
{
  CHASSIS_VECTOR_ZERO_FORCE,            //��������, ��û�ϵ�����
  CHASSIS_STOP,   											//����ǿ��ֹͣ
  CHASSIS_NORMAL_SPEED,                	//����ģʽ
	CHASSIS_LOW_SPEED,										//����ģʽ
	CHASSIS_HIGH_SPEED										//����ģʽ
} chassis_behaviour_e;

/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
  * @retval         none
  */

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif

