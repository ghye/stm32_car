#include "stdint.h"
#include "projects_conf.h"

extern volatile unsigned int Timer1, Timer2;

__asm void SystemReset(void) 
{ 
	MOV R0, #1           //;  
	MSR FAULTMASK, R0    //; 清除FAULTMASK 禁止一切中断产生 
	LDR R0, =0xE000ED0C  //; 
	LDR R1, =0x05FA0004  //; 
	STR R1, [R0]         //; 系统软件复位    

deadloop 
	B deadloop        //; 死循环使程序运行不到下面的代码 
}

int main(void)
{
	uint32_t reboot_cnt = 0;
	
	board_init();
	Timer1 = 200;
	while(Timer1);
	
  while (1)
  {
#if (0)
	
	while (1) {
		ret = lw_get_frame2sd();
		if (-1 == ret) {
			err_cnt++;
			debug_printf_s("-----------------------------------------");
		}
		else if (0 == ret) {
			ok_cnt++;
		}
		if (!((err_cnt + ok_cnt)%5)) {
			debug_printf_s("file err cnt:");
			debug_printf_h(err_cnt);
			debug_printf_s("file ok cnt:");
			debug_printf_h(ok_cnt);
			debug_printf_m("");
		}
		delay(30);
	}
	
#endif
	
	while(1){
		if (reboot_cnt++>3600) {
			SystemReset();
			//__set_FAULTMASK(1);
			//NVIC_SystemReset();
			while(1) ;
		}
		Timer1 = 100;
		while(Timer1) {
			#if defined(STM_SHIP)
			ais_seqed_proc();
			#endif
		}

		debug_printf_m("start loop");
		lw_gps_parse();
		lw_gprs_send_data();
		debug_printf_m("finish loop");
	}

  }
}

