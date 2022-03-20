#include "cmsis_os.h"
#include "DVTask.h"

uint8_t u1_buf[512];

void DVTask(void const * argument)
{
	while(1)
	{
		osDelay(1000);
	}
}