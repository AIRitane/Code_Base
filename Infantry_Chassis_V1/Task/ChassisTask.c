#include "ChassisTask.h"
#include "cmsis_os.h"
#include "CMSInterface.h"
#include "BspMotor.h"
#include "arm_math.h"
#include "ChassisPowerBehaviour.h"
<<<<<<< Updated upstream
=======
#include "math.h"
#include "user_lib.h"
<<<<<<< Updated upstream
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes

ChassisCtrl_t ChassisCtrl;
float XYPid[4][3]={{15000,0,0},
					{15000,0,0},
					{15000,0,0},
					{15000,0,0}};
float WZPid[3] = {0.03,0,0};

<<<<<<< Updated upstream
=======
BufferFunction_t BufferFunctionX;
BufferFunction_t BufferFunctionY;
BufferFunction_t BufferFunctionWZ;


void BufferFunctionInit(BufferFunction_t *BufferFunction,fp32 frame_period);
void BufferFunctionCalc(BufferFunction_t *BufferFunction,fp32 input);
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
		//APP_BatteryCombineBuckBoost2();
		
		ChassisPowerControl(&ChassisCtrl);
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
	ChassisCtrl.Mode = STOP;
=======
	ChassisCtrl.Mode = NOFORCE;
	BufferFunctionInit(&BufferFunctionX,200);
	BufferFunctionInit(&BufferFunctionY,200);
	BufferFunctionInit(&BufferFunctionWZ,500);
	
<<<<<<< Updated upstream
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
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
float ErrorAngle = 0;
void ChassisContolSet()
{
	float del = 0;
	int LsatMode = NOFORCE;

	uint8_t RotingFlag = 0;
	del = FallowAngle - ChassisCtrl.Yaw->angle;
<<<<<<< Updated upstream

	ChassisCtrl.vx = -PTZ.FBSpeed/32767.f * arm_cos_f32(del/180*PI) + PTZ.LRSpeed/32767.f * arm_sin_f32(del/180*PI);
	ChassisCtrl.vy = PTZ.FBSpeed/32767.f * arm_sin_f32(del/180*PI) + PTZ.LRSpeed/32767.f * arm_cos_f32(del/180*PI);
	
	if(ChassisCtrl.Mode == ROTING)
		ChassisCtrl.wz = RotingSpeed;
	else if(ChassisCtrl.Mode == FALLOW|
=======
	
	BufferFunctionCalc(&BufferFunctionX,PTZ.FBSpeed/32767.f);
	BufferFunctionCalc(&BufferFunctionY,PTZ.LRSpeed/32767.f);
	ChassisCtrl.vx = -BufferFunctionX.out * arm_cos_f32(del/180*PI) + BufferFunctionY.out * arm_sin_f32(del/180*PI);
	ChassisCtrl.vy = BufferFunctionX.out * arm_sin_f32(del/180*PI) + BufferFunctionY.out * arm_cos_f32(del/180*PI);
	
	if(ChassisCtrl.Mode == ROTING)
	{
		BufferFunctionCalc(&BufferFunctionWZ,RotingBaseSpeed);
		ChassisCtrl.wz = BufferFunctionWZ.out;
		LsatMode = ROTING;
		RotingFlag = 1;
	}
	/*-------------------------这里刹车代码待改进------------------------------------*/
	else if(ChassisCtrl.Mode == FALLOW||
<<<<<<< Updated upstream
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
		ChassisCtrl.Mode == STOP)
	{
		ErrorAngle = theta_format(del);
		
		if(ChassisCtrl.Mode == FALLOW||ChassisCtrl.Mode == STOP)
		{
			if(fabs(ErrorAngle) < 20)
			{
				ChassisCtrl.wz =  PID_calc(&ChassisCtrl.WZPid,ChassisCtrl.Yaw->angle,FallowAngle);
			}
			else
			{
				if(ErrorAngle>0)
				ChassisCtrl.wz = RotingBaseSpeed * 0.7;
				else
					ChassisCtrl.wz = -RotingBaseSpeed * 0.7;
			}
		}
	}
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
<<<<<<< Updated upstream
	
=======

void BufferFunctionInit(BufferFunction_t *BufferFunction,fp32 frame_period)
{
	BufferFunction->frame_period = frame_period;
	BufferFunction->input = 0;
	BufferFunction->out = 0;
	BufferFunction->error = 0;
	BufferFunction->buffer = 0;
}

void BufferFunctionCalc(BufferFunction_t *BufferFunction,fp32 input)
{
	BufferFunction->input = input;
	BufferFunction->error = BufferFunction->input - BufferFunction->out;
	if(BufferFunction->error >= 0)
	{
		BufferFunction->buffer = BufferFunction->error / (exp (0.7*BufferFunction->error));//0.7越小越往上飞
		BufferFunction->out += BufferFunction->buffer / BufferFunction->frame_period;
	}
	else
	{
		BufferFunction->buffer = -BufferFunction->error / (exp (-0.7*BufferFunction->error));//0.7越小越往上飞
		BufferFunction->out -= BufferFunction->buffer / BufferFunction->frame_period;
	}
}
>>>>>>> Stashed changes
