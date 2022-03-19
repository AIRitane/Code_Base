#include "cmsis_os.h"
#include "DVTask.h"

void DVTask(void const * argument)
{
	while(1)
	{
		osDelay(1000);
	}
}