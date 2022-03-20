#ifndef __DVTASK_H
#define __DVTASK_H

#include "struct_typedef.h"
#include "usart.h"
#include "stdio.h"

extern uint8_t u1_buf[512];
#define U1Printf(...)     HAL_UART_Transmit(&huart1,\
                                           (uint8_t *)u1_buf,\
                                           sprintf((char*)u1_buf,__VA_ARGS__),\
                                           0xffff)                                 
#define U1Printf_IT(...)    HAL_UART_Transmit_IT(&huart1,\
                                                (uint8_t *)u1_buf,\
                                                sprintf((char*)u1_buf,__VA_ARGS__)) 
#define U1Printf_DMA(...)   HAL_UART_Transmit_DMA(&huart1,\
                                                 (uint8_t *)u1_buf,\
                                                sprintf((char*)u1_buf,__VA_ARGS__))
													
#endif