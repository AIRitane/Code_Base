#include "ChassisTask.h"
#include "cmsis_os.h"
#include "CMSInterface.h"
#include "BspMotor.h"
#include "arm_math.h"
#include "ChassisPowerBehaviour.h"
#include "math.h"

ChassisCtrl_t ChassisCtrl;
float XYPid[4][3]={{15000,0,0},
					{15000,0,0},
					{15000,0,0},
					{15000,0,0}};
float WZPid[3] = {0.03,0,0};

BufferFunction_t BufferFunctionX;
BufferFunction_t BufferFunctionY;

void BufferFunctionInit(BufferFunction_t *BufferFunction,fp32 frame_period);
void BufferFunctionCalc(BufferFunction_t *BufferFunction,fp32 input);

void ChassisInit();
void ChassisSetmode();
void ChassisContolSet();
void ChassisControlLoop();

void ChassisTask(void const * argument)
{
	osDelay(10);
	
	ChassisInit();
	
	while(1)
	{
		ChassisSetmode();
		ChassisContolSet();
		//APP_BatteryCombineBuckBoost2();
		ChassisControlLoop();
		ChassisPowerControl(&ChassisCtrl);
		
		for(int i=0;i<4;i++)
		{
			ChassisCtrl.WheelSpeed[i]*=SupKp;
		}
		ChassisControlLoop();
		
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
	ChassisCtrl.Mode = NOFORCE;
	BufferFunctionInit(&BufferFunctionX,1000);
	BufferFunctionInit(&BufferFunctionY,1000);
}
void ChassisSetmode()
{
	switch(PTZ.ChassisStatueRequest)
	{
		case 0x01:
			ChassisCtrl.Mode = NOFORCE; break;
		case 0x12:
			ChassisCtrl.Mode = ROTING; break;
		case 0x0A:
			ChassisCtrl.Mode = FALLOW; break;
		case 0x06:
			ChassisCtrl.Mode = STOP; break;
		default:
		{
			ComLedError();
			break;
		}	
	}
}
void ChassisContolSet()
{
	float del = 0;
	del = FallowAngle - ChassisCtrl.Yaw->angle;
	
	BufferFunctionCalc(&BufferFunctionX,PTZ.FBSpeed/32767.f);
	BufferFunctionCalc(&BufferFunctionY,PTZ.LRSpeed/32767.f);
	ChassisCtrl.vx = -BufferFunctionX.out * arm_cos_f32(del/180*PI) + BufferFunctionY.out/32767.f * arm_sin_f32(del/180*PI);
	ChassisCtrl.vy = BufferFunctionX.out/32767.f * arm_sin_f32(del/180*PI) + BufferFunctionY.out/32767.f * arm_cos_f32(del/180*PI);
	
	if(ChassisCtrl.Mode == ROTING)
		ChassisCtrl.wz = RotingBaseSpeed;
	else if(ChassisCtrl.Mode == FALLOW|
		ChassisCtrl.Mode == STOP)
		ChassisCtrl.wz =  PID_calc(&ChassisCtrl.WZPid,ChassisCtrl.Yaw->angle,FallowAngle);
}

void ChassisControlLoop()
{
	if(ChassisCtrl.Mode == NOFORCE)
	{
		memset(ChassisCtrl.Current,0,sizeof(ChassisCtrl.Current));
		for(int i = 0;i<4;i++)
		{
			if(ChassisCtrl.Current[i] != 0)
			{
				ChassisCtrl.Current[i] = 0;
				TestLedError();
			}
				
		}
		return;
	}
	
	else if(ChassisCtrl.Mode == STOP)
	{
		memset(ChassisCtrl.WheelSpeed,0,sizeof(ChassisCtrl.WheelSpeed));
		for(int i = 0;i<4;i++)
		{
			if(ChassisCtrl.WheelSpeed[i] != 0)
			{
				ChassisCtrl.WheelSpeed[i] = 0;
				TestLedError();
			}
				
		}
		for(int i =0;i<4;i++)
		{
			ChassisCtrl.Current[i] =  PID_calc(&ChassisCtrl.XYPid[i], ChassisCtrl.Motor[i]->speed_rpm * 0.000415809748903494517209f, ChassisCtrl.WheelSpeed[i]);
		}
	}
	else if(ChassisCtrl.Mode == FALLOW || ChassisCtrl.Mode == ROTING)
	{
		ChassisCtrl.WheelSpeed[0] = -ChassisCtrl.vx - ChassisCtrl.vy + ChassisCtrl.wz;
		ChassisCtrl.WheelSpeed[1] = ChassisCtrl.vx - ChassisCtrl.vy +  ChassisCtrl.wz;
		ChassisCtrl.WheelSpeed[2] = ChassisCtrl.vx + ChassisCtrl.vy + 	ChassisCtrl.wz;
		ChassisCtrl.WheelSpeed[3] = -ChassisCtrl.vx + ChassisCtrl.vy + ChassisCtrl.wz;
		
		 
		for(int i =0;i<4;i++)
		{
			ChassisCtrl.WheelSpeed[i]*=3;
			ChassisCtrl.Current[i] =  PID_calc(&ChassisCtrl.XYPid[i], ChassisCtrl.Motor[i]->speed_rpm * 0.000415809748903494517209f, ChassisCtrl.WheelSpeed[i]);
		}
	}
}

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
	if(BufferFunction->input >= 0)
	{
		BufferFunction->error = BufferFunction->input - BufferFunction->out;
		BufferFunction->buffer = BufferFunction->error / (exp (0.7*BufferFunction->error));//0.7越小越往上飞
		BufferFunction->out += BufferFunction->buffer / BufferFunction->frame_period;
	}
	else
	{
		BufferFunction->error = fabs(BufferFunction->input - BufferFunction->out);
		BufferFunction->buffer = BufferFunction->error / (exp (0.7*BufferFunction->error));//0.7越小越往上飞
		BufferFunction->out -= BufferFunction->buffer / BufferFunction->frame_period;
	}
}