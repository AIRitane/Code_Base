#include "cmsis_os.h"
#include "DVTask.h"

uint8_t u1_buf[512];

void DVTask(void const * argument)
{
	while(1)
	{
<<<<<<< Updated upstream
		osDelay(1000);
=======
		/*---------------------�˴������⣬һ�����������������������----------------------------------*/
		//U1Printf_DMA("%f,%f,%f\n",SupKp,total_current_limit,total_current);
		osDelay(100);
>>>>>>> Stashed changes
	}
}