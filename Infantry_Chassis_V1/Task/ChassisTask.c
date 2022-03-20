#include "ChassisTask.h"
#include "cmsis_os.h"
#include "CMSInterface.h"
#include "BspMotor.h"
#include "arm_math.h"
#include "ChassisPowerBehaviour.h"

ChassisCtrl_t ChassisCtrl;
float XYPid[4][3]={{15000,0,0},
					{15000,0,0},
					{15000,0,0},
					{15000,0,0}};
float WZPid[3] = {0.03,0,0};


void ChassisInit();
void ChassisSetmode();
void ChassisContolSet();
void ChassisControlLoop();
	
void ChassisTask(void const * argument)
{
	ChassisInit();
	
	while(1)
	{
		ChassisSetmode();
		ChassisContolSet();
		ChassisControlLoop();
		//APP_BatteryCombineBuckBoost2();
		
		ChassisPowerControl(&ChassisCtrl);
		ChassisCMD(ChassisCtrl.Current[0], ChassisCtrl.Current[1], ChassisCtrl.Current[2], ChassisCtrl.Current[3]);
		
		osDelay(1);
	}
}

void ChassisInit()
{
	for(int i=0;i<4;i++) 
	{
		ChassisCtrl.Current[i] = 0;
		ChassisCtrl.Motor[i] = GetChassisMeasure(i);
		PID_init(&ChassisCtrl.XYPid[i], PID_POSITION, XYPid[i], 16300, 5000);	
	}

	ChassisCtrl.Yaw = GetYawMeasure();
	PID_init(&ChassisCtrl.WZPid,PID_ANGLE,WZPid,1,0);
	ChassisCtrl.Mode = STOP;
}
void ChassisSetmode()
{
	switch(PTZ.ChassisStatueRequest)
	{
		case 0x00:
			ChassisCtrl.Mode = NOFORCE; break;
		case 0x12:
			ChassisCtrl.Mode = ROTING; break;
		case 0x0A:
			ChassisCtrl.Mode = FALLOW; break;
		case 0x06:
			ChassisCtrl.Mode = STOP; break;
		case 0x01:
			ChassisCtrl.Mode = STOP; break;
		default:
			break;
	}
}
void ChassisContolSet()
{
	float del = 0;
	del = FallowAngle - ChassisCtrl.Yaw->angle;

	ChassisCtrl.vx = -PTZ.FBSpeed/32767.f * arm_cos_f32(del/180*PI) + PTZ.LRSpeed/32767.f * arm_sin_f32(del/180*PI);
	ChassisCtrl.vy = PTZ.FBSpeed/32767.f * arm_sin_f32(del/180*PI) + PTZ.LRSpeed/32767.f * arm_cos_f32(del/180*PI);
	
	if(ChassisCtrl.Mode == ROTING)
		ChassisCtrl.wz = RotingSpeed;
	else if(ChassisCtrl.Mode == FALLOW|
		ChassisCtrl.Mode == STOP)
		ChassisCtrl.wz =  PID_calc(&ChassisCtrl.WZPid,ChassisCtrl.Yaw->angle,FallowAngle);
}

float wheel_speed[4];
void ChassisControlLoop()
{
	if(ChassisCtrl.Mode == NOFORCE)
	{
		for(int i = 0;i<4;i++)
			ChassisCtrl.Current[i] = 0;
		return;
	}
	
	else if(ChassisCtrl.Mode == STOP)
	{
		memset(wheel_speed,0,sizeof(wheel_speed));
		for(int i =0;i<4;i++)
		{
			ChassisCtrl.Current[i] =  PID_calc(&ChassisCtrl.XYPid[i], ChassisCtrl.Motor[i]->speed_rpm * 0.000415809748903494517209f, wheel_speed[i]);
		}
	}
	else if(ChassisCtrl.Mode == FALLOW || ChassisCtrl.Mode == ROTING)
	{
		wheel_speed[0] = -ChassisCtrl.vx - ChassisCtrl.vy + ChassisCtrl.wz;
		wheel_speed[1] = ChassisCtrl.vx - ChassisCtrl.vy +  ChassisCtrl.wz;
		wheel_speed[2] = ChassisCtrl.vx + ChassisCtrl.vy + 	ChassisCtrl.wz;
		wheel_speed[3] = -ChassisCtrl.vx + ChassisCtrl.vy + ChassisCtrl.wz;
		
		 
		for(int i =0;i<4;i++)
		{
			//wheel_speed[i]*=4;
			ChassisCtrl.Current[i] =  PID_calc(&ChassisCtrl.XYPid[i], ChassisCtrl.Motor[i]->speed_rpm * 0.000415809748903494517209f, wheel_speed[i]);
		}
	}
}
	
