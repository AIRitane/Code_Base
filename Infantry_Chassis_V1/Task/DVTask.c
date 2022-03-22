#include "cmsis_os.h"
#include "DVTask.h"
#include "ChassisTask.h"
#include "ChassisPowerBehaviour.h"

uint8_t u1_buf[512];
extern ChassisCtrl_t ChassisCtrl;
void DVTask(void const * argument)
{
	osDelay(1000);
	while(1)
	{
		U1Printf_DMA("%f\n",SupKp);
		osDelay(100);
	}
}