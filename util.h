#ifndef __UTIL_H__
#define __UTIL_H__

#include "lw_stm32_uart.h"

/*****************延迟函数********************************/
/*---------------------------------
函数名：延时调整形式的delaynus函数, 
描  述：参数1即为1ms
-----------------------------------*/
void delayms(unsigned int ms);
/*---------------------------------
函数名：延时调整形式的delay函数, 
描  述：参数1即为1s
-----------------------------------*/
void delay(unsigned long n);

void debug_printf_m(uint8_t *m) ;
void debug_printf_h(uint8_t m);
void debug_printf_s(uint8_t *m);

#endif
