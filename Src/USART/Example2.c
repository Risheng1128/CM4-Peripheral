/**
  ******************************************************************************
  * @file    Example2.c
  * @author  Ri-Sheng Chen
  * @brief   This file is a simple usart example
  ******************************************************************************
  * @attention
  *    1. 使用USART3將資料傳到PC並顯示
  **/

#include "myusart.h"
#include <stdio.h>

int main(void) 
{
	MYUSART_Init();
	int data;
	while(1) 
	{
		printf("input a data!!\n");
		scanf("%d", &data);
		printf("data = %d\n", data);
		fflush(stdin);	/* Claer buffer */
	}
	return 0;
}