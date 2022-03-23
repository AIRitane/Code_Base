/*
* 该代码仅测试了在60W的限制下的电流最大值，250缓存以及更高的电压都并未测试
*
*
*/

#include "ChassisPowerBehaviour.h"
#include "RefereeBehaviour.h"
#include "arm_math.h"
#include "CMSInterface.h"

//缓存功率限制内的最大总电流
#define BUFFER_TOTAL_CURRENT_LIMIT      14000.0f
//最大功率限制内的最大总电流
#define POWER_TOTAL_CURRENT_LIMIT       50000.0f

float BufferEnergy = 60; //底盘缓冲能量
uint8_t RfidFlag = 0;

float SupKp = 1;
uint8_t CalculCount = 0;

//计算得到
float remain_current;				//当前剩余给超级电容的电流
float available_current;			//当前可用电流


//裁判系统得到数值
fp32 chassis_volt = 0;   //单位是V
float power_limit = 0;
fp32 chassis_power = 0.0f;
fp32 chassis_power_buffer = 0.0f;

//电流限制
fp32 LimitCurrent_temp = 0;//超级电容充电电流
fp32 total_current_limit = 0.0f;
fp32 total_current = 0.0f;

void ChassisPowerInit();
void ChassisBufferEnerge();
void CapCharge(float Percentage);
void ChassisReduceRate();
void GetSupKp(ChassisCtrl_t *ChassisCtrl);
void ChassisPowerControl(ChassisCtrl_t *ChassisCtrl);


void ChassisPowerInit()
{
	chassis_power = power_heat_data_t.chassis_power;
	chassis_power_buffer = power_heat_data_t.chassis_power_buffer;
	
	chassis_volt = (fp32)power_heat_data_t.chassis_volt / 1000;
	power_limit = robot_state.chassis_power_limit;
	available_current = power_limit/ chassis_volt;
	remain_current = available_current - (fp32)power_heat_data_t.chassis_current / 1000;
	if(remain_current<0)
	{
		remain_current = 0;
	}
	total_current = 0;
	total_current_limit = 0;
}
void ChassisBufferEnerge()
{
	if((rfid_status_t.rfid_status&0x03) ^ RfidFlag)
	{
		TestLedError();
		if((rfid_status_t.rfid_status&0x03) == 1)
		{
			BufferEnergy = 250;
			RfidFlag = 1;
		}
		else
		{
			BufferEnergy = 60;
			RfidFlag = 0;
		}		
	}
	if(chassis_power_buffer <=60 ) BufferEnergy = 60;
}

void CapCharge(float Percentage)
{
	LimitCurrent_temp = remain_current * 100 * Percentage;
	CMS_Hub.LimitCurrent = (uint16_t)LimitCurrent_temp ;
}

void ChassisReduceRate()
{
	//电容供电,不限制
	if(CMS_Hub.power_routin == CMS_PR_BuckBoost && (CMS_Hub.CMS_ChagingPower>14.5))
	{
		total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
	}
	//电池供电，判断是否超缓存
	else if(BufferEnergy == 60)
	{
PowerControlError:
		if(chassis_power_buffer != BufferEnergy)
		{
			float ReduceScale = 0;
			if(chassis_power_buffer > 10.0f)
            {
                ReduceScale = chassis_power_buffer / BufferEnergy;
            }
            else
            {
                ReduceScale = 0.0f;
            }
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * ReduceScale;
		}
		else
		{
			uint32_t WarnPower = 0;
			WarnPower =  power_limit * 0.70f;
			
			if(chassis_power > WarnPower)
            {
                fp32 power_scale;
                power_scale = (power_limit - chassis_power) / (power_limit - WarnPower);
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
		}
	}
	else if(BufferEnergy == 250)
	{
		if(chassis_power_buffer < BufferEnergy && chassis_power_buffer>60)
		{
			/*--------------------------------------待处理---------------------------------------------*/
			TestLedError();
		}
		else if(chassis_power_buffer <60)
		{
			/*--------------------------------------待处理---------------------------------------------*/
			TestLedError();
		}
		else
		{
			/*--------------------------------------待处理---------------------------------------------*/
			TestLedError();
		}
	}
	else 
	{
		BufferEnergy = 60;
		goto PowerControlError;
		CtrlLedError();
	}
}

float currut;
void GetSupKp(ChassisCtrl_t *ChassisCtrl)
{
	float KpW = 0,KpEi = 0;
	
	for(int i=0;i<4;i++)
	{
		total_current += fabs(ChassisCtrl->Current[i]);
	}
		
	if(total_current > total_current_limit)
    {
			if(ChassisCtrl->WheelSpeed[0] == 0 &&
				ChassisCtrl->WheelSpeed[1] == 0 &&
				ChassisCtrl->WheelSpeed[2] == 0 &&
				ChassisCtrl->WheelSpeed[3] == 0)
			//这种情况必须不应该出现，因为添加了缓冲函数，若出现，不管电压，直接使用超级电容
			//即使疲软，也没办法
			{
				TestLedError();
			}
			else
			{
				for(int i = 0; i < 4; i++)
				{
					if(ChassisCtrl->XYPid[i].error[0]>0)
					{
						KpEi += (ChassisCtrl->Motor[i]->speed_rpm * 0.000415809748903494517209f)* ChassisCtrl->XYPid[i].Kp;
						KpW += ChassisCtrl->WheelSpeed[i] * ChassisCtrl->XYPid[i].Kp;
					}
					if(ChassisCtrl->XYPid[i].error[0]<=0)
					{
						KpEi -= (ChassisCtrl->Motor[i]->speed_rpm * 0.000415809748903494517209f)* ChassisCtrl->XYPid[i].Kp;
						KpW -= ChassisCtrl->WheelSpeed[i]* ChassisCtrl->XYPid[i].Kp;
					}
				}
				
			}
		SupKp = (KpEi + total_current_limit)/KpW;
    }
	else
	{
		SupKp = 1;
	}
}

float copy;
void ChassisPowerControl(ChassisCtrl_t *ChassisCtrl)
{
	ChassisPowerInit();
	ChassisBufferEnerge();
	CapCharge(0.5);
	ChassisReduceRate();
	GetSupKp(ChassisCtrl);
	
	
//	float KpW = 0,KpEi = 0;
//	for(int i = 0; i < 4; i++)
//				{
//					if(ChassisCtrl->XYPid[i].error[0]>0)
//					{
//						KpEi += (ChassisCtrl->Motor[i]->speed_rpm * 0.000415809748903494517209f)* ChassisCtrl->XYPid[i].Kp;
//						KpW += ChassisCtrl->WheelSpeed[i] * ChassisCtrl->XYPid[i].Kp;
//					}
//					if(ChassisCtrl->XYPid[i].error[0]<=0)
//					{
//						KpEi -= (ChassisCtrl->Motor[i]->speed_rpm * 0.000415809748903494517209f)* ChassisCtrl->XYPid[i].Kp;
//						KpW -= ChassisCtrl->WheelSpeed[i]* ChassisCtrl->XYPid[i].Kp;
//					}
//				}
//				
//			
//			currut = (KpW - KpEi);
//			copy = total_current;
}
