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
		/*---------------------此处有问题，一旦开启，其他串口任务祭天----------------------------------*/
		//U1Printf_DMA("%f,%f,%f\n",SupKp,total_current_limit,total_current);
		osDelay(100);
>>>>>>> Stashed changes
	}
}