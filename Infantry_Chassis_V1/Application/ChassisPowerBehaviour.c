/*
* �ô������������60W�������µĵ������ֵ��250�����Լ����ߵĵ�ѹ����δ����
*
*
*/

#include "ChassisPowerBehaviour.h"
#include "RefereeBehaviour.h"
#include "arm_math.h"
#include "CMSInterface.h"

//���湦�������ڵ�����ܵ���
#define BUFFER_TOTAL_CURRENT_LIMIT      14000.0f
//����������ڵ�����ܵ���
#define POWER_TOTAL_CURRENT_LIMIT       50000.0f

float BufferEnergy = 60; //���̻�������
uint8_t RfidFlag = 0;

float SupKp = 1;
uint8_t CalculCount = 0;

//����õ�
float remain_current;				//��ǰʣ����������ݵĵ���
float available_current;			//��ǰ���õ���


//����ϵͳ�õ���ֵ
fp32 chassis_volt = 0;   //��λ��V
float power_limit = 0;
fp32 chassis_power = 0.0f;
fp32 chassis_power_buffer = 0.0f;

//��������
fp32 LimitCurrent_temp = 0;//�������ݳ�����
fp32 total_current_limit = 0.0f;
fp32 total_current = 0.0f;

void ChassisPowerInit();
void ChassisBufferEnerge();
void CapCharge(float Percentage);
void ChassisReduceRate();
uint8_t GetSupKp(ChassisCtrl_t *ChassisCtrl);
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
	//���ݹ���,������
	if(CMS_Hub.power_routin == CMS_PR_BuckBoost && (CMS_Hub.CMS_ChagingPower>14.5))
	{
		total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
	}
	//��ع��磬�ж��Ƿ񳬻���
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
			/*--------------------------------------������---------------------------------------------*/
			TestLedError();
		}
		else if(chassis_power_buffer <60)
		{
			/*--------------------------------------������---------------------------------------------*/
			TestLedError();
		}
		else
		{
			/*--------------------------------------������---------------------------------------------*/
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

static int Counter = 0;
uint8_t GetSupKp(ChassisCtrl_t *ChassisCtrl)
{
	total_current = 0 ;
	
	for(int i=0;i<4;i++)
	{
		total_current += fabs(ChassisCtrl->Current[i]);
	}

	if(total_current > total_current_limit)
    {
		SupKp = total_current_limit/total_current;
		for(int i=0;i<4;i++)
		{
			ChassisCtrl->WheelSpeed[i] *= SupKp;
		}
		ChassisControlLoop();
		Counter++;
		if(Counter > 3)
		{
			Counter  = 0;
			for(int i=0;i<4;i++)
			{
				ChassisCtrl->Current[i] *= SupKp;
			}
			return 0;
		}
		return 1;
    }
	else
	{
		Counter = 0;
		SupKp = 1;
		return 0;
	}
}


void ChassisPowerControl(ChassisCtrl_t *ChassisCtrl)
{
	ChassisPowerInit();
	ChassisBufferEnerge();
	CapCharge(0.5);
	ChassisReduceRate();
	while(GetSupKp(ChassisCtrl));
}
