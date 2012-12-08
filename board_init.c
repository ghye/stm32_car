
#include "stm32f10x_flash.h"
#include "stm32f10x.h" 
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"


#include "lw_gprs.h"
#include "lw_gps.h"
#include "lw_sd.h"
#include "log.h"
#include "lw_stm32_spi.h"
#include "lw_vc0706.h"
#include "ctrl_gps_cam.h"
#include "lw_ais.h"
#include "lw_stm32_dma.h"
#include "projects_conf.h"
#include "save_jpg_algorithm.h"
#include "lw_stm32_wdg.h"

void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
#ifdef VECT_TAB_RAM
    // Set the Vector Tab base at location at 0x20000000
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else
    // Set the Vector Tab base at location at 0x80000000
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
    /* Configure the NVIC Preemption Priority Bits[配置优先级组] */
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the TIM2 gloabal Interrupt [允许TIM2全局中断]*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//Channel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void TIM2_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    //  TIM_OCInitTypeDef  TIM_OCInitStructure ;
    TIM_DeInit( TIM2);                              //复位TIM2定时器

    /* TIM2 clock enable [TIM2定时器允许]*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* TIM2 configuration */
    TIM_TimeBaseStructure.TIM_Period = 20;          //
    TIM_TimeBaseStructure.TIM_Prescaler = 53999; //35999;    // 108M/(53999+1)/20 = 100Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //计数方向向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* Clear TIM2 update pending flag[清除TIM2溢出中断标志] */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    /* Enable TIM2 Update interrupt [TIM2溢出中断允许]*/
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* TIM2 enable counter [允许tim2计数]*/
    TIM_Cmd(TIM2, ENABLE);
}

void board_init(void)
{
	NVIC_Config();
	TIM2_Config();
	
	spi_init();
	
	lw_gprs_init();
	lw_start_gprs_mode();
	lw_seqed_msgs_init();
	
	lw_gps_init();
	lw_gps_param_init();

	lw_sd_fatfs_init();
	
#if defined (STM_SHIP)	
	ais_global_init();
	ais_rx_init();
	ais_rx_tx_dma_init();
	ais_rx_set_dma();
	ais_tx_set_dma();
#else
	lw_vc0706_init();
	lw_vc0706_param_init();	

	get_memory_most_from_sd();
	send_jpg_from_sd_global_init();
#endif

	ctrl_gps_cam_init();

	log_open();

	wdg_init();
	wdg_start();
}

