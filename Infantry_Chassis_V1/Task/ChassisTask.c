#include "ChassisTask.h"
#include "cmsis_os.h"
#include "CMSInterface.h"

void ChassisTask(void const * argument)
{
	while(1)
	{
		APP_BatteryCombineBuckBoost2();
		osDelay(1);
	}
}
