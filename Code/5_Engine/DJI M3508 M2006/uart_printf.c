/**
 * @file uart_printf.c
 * @author TITH (1023515576@qq.com)
 * @brief printf()串口重定向
 * @version 1.0
 * @date 2022-01-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "usart.h"
#include <stdio.h>

// 串口号配置
#define UART_Printf_Config_huart huart1

/* 重定向printf */

// 对于 GCC: 

int _write(int fd, char *pBuffer, int size)  
{  
	while (HAL_UART_Transmit(&UART_Printf_Config_huart, (uint8_t *)pBuffer, size, 0xffff) == HAL_BUSY);
	return size;  
}

// 对于 MDK: 

int fputc(int ch, FILE *stream)
{
	while (HAL_UART_Transmit(&UART_Printf_Config_huart, (uint8_t *)&ch, 1, 0xffff) == HAL_BUSY);
	return ch;
}