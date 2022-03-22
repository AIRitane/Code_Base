/*
* �ô������������60W�������µĵ������ֵ��250�����Լ����ߵĵ�ѹ����δ����
*
*
*/

#include "ChassisPowerBehaviour.h"
#include "RefereeBehaviour.h"
#include "arm_math.h"
#include "CMSInterface.h"


#define WARNING_POWER_BUFF  		60.0f   			//���̻��幦��
//���湦�������ڵ�����ܵ���
#define BUFFER_TOTAL_CURRENT_LIMIT      14000.0f
//����������ڵ�����ܵ���
#define POWER_TOTAL_CURRENT_LIMIT       50000.0f

uint32_t warning_power ;  
//����õ�
float remain_current;				//��ǰʣ����������ݵĵ���
float available_current;			//��ǰ���õ���
fp32 chassis_power = 0.0f;
fp32 chassis_power_buffer = 0.0f;

//����ϵͳ�õ���ֵ
fp32 chassis_volt = 0;   //��λ��V
float power_limit ;  

fp32 LimitCurrent_temp = 0;


void ChassisPowerControl(ChassisCtrl_t *ChassisCtrl);

void ChassisPowerControl(ChassisCtrl_t *ChassisCtrl)
{
	fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
	
	chassis_volt = (fp32)power_heat_data_t.chassis_volt / 1000;
	power_limit = robot_state.chassis_power_limit;
	available_current = (power_limit+ WARNING_POWER_BUFF) / chassis_volt;
	remain_current = available_current - (fp32)power_heat_data_t.chassis_current / 1000;
	
	if(remain_current<0)
		remain_current = 0;
	
	LimitCurrent_temp = remain_current * 100 * 0.5;	//��ǰ���������ݹ���ĵ���
	
	if(CMS_Hub.power_routin == CMS_PR_BuckBoost && (CMS_Hub.CMS_ChagingPower>14.5))//���ݹ���
	{
		total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
	}
	else 
	{
		chassis_power = power_heat_data_t.chassis_power;
		chassis_power_buffer = power_heat_data_t.chassis_power_buffer;
		
        if(chassis_power_buffer < WARNING_POWER_BUFF)//250�ߵ�����ų�
        {
			LimitCurrent_temp = 0;
			
            fp32 power_scale;
			//��ʣ�໺�幦�ʳ���10ʱ
            if(chassis_power_buffer > 10.0f)
            {
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                power_scale = 0.0f;
            }
			
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
		else
        {
			warning_power =  power_limit * 0.70f;
			
            if(chassis_power > warning_power)
            {
                fp32 power_scale;
                if(chassis_power < power_limit)
                {
                    power_scale = (power_limit - chassis_power) / (power_limit - warning_power);
                }
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
<<<<<<< Updated upstream
        }

   }
	CMS_Hub.LimitCurrent = (uint16_t)LimitCurrent_temp ;							//��������ʽת��Ϊ�������ذ�Ŀ��Ƹ�ʽ
    
    total_current = 0.0f;

    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(ChassisCtrl->Current[i]);
=======
		}
	}
	else if(BufferEnergy == 250)
	{
		if(chassis_power_buffer < BufferEnergy)
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
void GetSupKp(ChassisCtrl_t *ChassisCtrl)
{
	float KpW = 0,KpEi = 0;
	
	total_current = 0;
	
	for(int i=0;i<4;i++)
		total_current += ChassisCtrl->Current[i];
	
	if(fabs(total_current) > total_current_limit)
    {
			if(ChassisCtrl->WheelSpeed[0] == 0 &&
				ChassisCtrl->WheelSpeed[1] == 0 &&
				ChassisCtrl->WheelSpeed[2] == 0 &&
				ChassisCtrl->WheelSpeed[3] == 0)
			//����������벻Ӧ�ó��֣���Ϊ����˻��庯���������֣����ܵ�ѹ��ֱ��ʹ�ó�������
			//��ʹƣ��Ҳû�취
			{
				TestLedError();
			}
			else
			{
				for(int i = 0; i < 4; i++)
				{
					KpEi += ChassisCtrl->XYPid->Kp * ChassisCtrl->XYPid->error[0];
					KpW += ChassisCtrl->XYPid->Kp * ChassisCtrl->WheelSpeed[i];
				}
			}
		SupKp = (total_current_limit+KpEi)/KpW;
>>>>>>> Stashed changes
    }
    
	//��С����������������������Ƶ���Խ�󣬳߶���СԽ����
    if(total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        ChassisCtrl->Current[0]*=current_scale;
        ChassisCtrl->Current[1]*=current_scale;
		ChassisCtrl->Current[2]*=current_scale;
		ChassisCtrl->Current[3]*=current_scale;
    }
		
}
	
