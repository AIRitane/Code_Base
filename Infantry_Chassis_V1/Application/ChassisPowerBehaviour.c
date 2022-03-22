/*
* 该代码仅测试了在60W的限制下的电流最大值，250缓存以及更高的电压都并未测试
*
*
*/

#include "ChassisPowerBehaviour.h"
#include "RefereeBehaviour.h"
#include "arm_math.h"
#include "CMSInterface.h"


#define WARNING_POWER_BUFF  		60.0f   			//底盘缓冲功率
//缓存功率限制内的最大总电流
#define BUFFER_TOTAL_CURRENT_LIMIT      14000.0f
//最大功率限制内的最大总电流
#define POWER_TOTAL_CURRENT_LIMIT       50000.0f

uint32_t warning_power ;  
//计算得到
float remain_current;				//当前剩余给超级电容的电流
float available_current;			//当前可用电流
fp32 chassis_power = 0.0f;
fp32 chassis_power_buffer = 0.0f;

//裁判系统得到数值
fp32 chassis_volt = 0;   //单位是V
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
	
	LimitCurrent_temp = remain_current * 100 * 0.5;	//当前给超级电容供电的电流
	
	if(CMS_Hub.power_routin == CMS_PR_BuckBoost && (CMS_Hub.CMS_ChagingPower>14.5))//电容供电
	{
		total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
	}
	else 
	{
		chassis_power = power_heat_data_t.chassis_power;
		chassis_power_buffer = power_heat_data_t.chassis_power_buffer;
		
        if(chassis_power_buffer < WARNING_POWER_BUFF)//250瓦的情况排除
        {
			LimitCurrent_temp = 0;
			
            fp32 power_scale;
			//当剩余缓冲功率超过10时
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
	CMS_Hub.LimitCurrent = (uint16_t)LimitCurrent_temp ;							//将电流格式转化为电容主控板的控制格式
    
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
			//这种情况必须不应该出现，因为添加了缓冲函数，若出现，不管电压，直接使用超级电容
			//即使疲软，也没办法
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
    
	//缩小底盘输出电流，当超过限制电流越大，尺度缩小越明显
    if(total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        ChassisCtrl->Current[0]*=current_scale;
        ChassisCtrl->Current[1]*=current_scale;
		ChassisCtrl->Current[2]*=current_scale;
		ChassisCtrl->Current[3]*=current_scale;
    }
		
}
	
