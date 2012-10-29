#include <string.h>

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


/*
	在buf里面查找符合str的一串数据
*/
uint8_t *memstr(uint8_t *buf, uint32_t buflen, uint8_t *str, uint32_t strlen)
{
	uint32_t i;
	uint8_t *p;

	while (buflen) {
		if ((p = memchr(buf, *str, buflen)) != NULL) {
			if (buflen - (p - buf) < strlen)
				return NULL;
			for (i=1; i<strlen; i++) {
				if (p[i] != str[i]){
					buflen -= p - buf;
					buf = p + 1;
					buflen -= 1;
					break;
				}
			}
			if (i == strlen) { /*find */
				return p;
			}
		}
		else {
			return NULL;
		}
	}

	return NULL;
}



