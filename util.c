#include "util.h"

/*****************延迟函数********************************/
/*---------------------------------
函数名：延时调整形式的delaynus函数, 
描  述：参数1即为1ms
-----------------------------------*/
void delayms(unsigned int ms) 
{
  unsigned char k;
  while(ms--)
  {
    for(k = 0; k < 114; k++);
  }
}
/*---------------------------------
函数名：延时调整形式的delay函数, 
描  述：参数1即为1s
-----------------------------------*/
void delay(unsigned long n)
{
 while(n--)
  delayms(1000);
}


void debug_printf_m(uint8_t *m) 
{
	com_send_message(2, m);
}

void debug_printf_h(uint8_t m)
{
	com_send_hex(2, m);
}

void debug_printf_s(uint8_t *m)
{
	com_send_string(2, m);
}




