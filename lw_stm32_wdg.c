/*
*/

#include "stm32f10x_iwdg.h"

void wdg_init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//启动寄存器读写

	IWDG_SetPrescaler(IWDG_Prescaler_256);//40K时钟256分频

	IWDG_SetReload(0xfff);                 //计数器数值，大概26秒

	//IWDG_ReloadCounter();             //重启计数器

	//IWDG_Enable();                       //启动看门狗
}

void  wdg_start(void)
{
	IWDG_ReloadCounter();             //重启计数器

	IWDG_Enable();                       //启动看门狗
}

void wdg_keeplive(void)
{
	IWDG_ReloadCounter();             //重启计数器
}

