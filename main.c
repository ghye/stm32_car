/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V1.0
* Date               : 10/08/2007
* Description        : Main program body
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
//#include <stm32f10x.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "stdint.h"
#include "stdbool.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
void Delay(vu32 nCount);
//******************************************************************************
// Function Name  : NVIC_Configuration
// Description    : Nested Vectored Interrupt Controller configuration
// Input          : None
// Output         : None
// Return         : None
//******************************************************************************
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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the TIM2 gloabal Interrupt [允许TIM2全局中断]*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//Channel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
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

#include "util.h"
#include "lw_stm32_uart.h"
void test_com_init(void)
{	
	com_para_t com_para;

	com_para.baudrate = 115200;
	com_para.data_9bit = false;
	com_para.parity = 0;
	com_para.port = 1;//2;
	com_para.twoStopBits = false;

	com_init(&com_para);
}

void test_com(void)
{
	com_send_char(1, 0xa5a5);
    	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	delay(2);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	delay(2);	
}


#include "lw_stm32_spi.h"
void test_spi_init()
{
	spi_init();
}

uint8_t sbuf[10];
uint8_t rbuf[10];
void test_spi()
{
	int i;
	int length;
com_send_char(1, 0xa5a5);
	for(i=0; i<sizeof(sbuf)/sizeof(sbuf[0]); i++)
	{
		sbuf[i] = 80+i;
	}
	spi_send(sbuf, sizeof(sbuf)/sizeof(sbuf[0]));
	#if 0
	length = spi_recv(rbuf);
	if(length > 0)
	{
		for(i=0; i<length; i++)
			com_send_char(1, rbuf[i]);		
	}
	else
	{
		/* FIXME: error msg */
	}
	#endif

	//com_send_char(1, 0xa5a5);
}

bool is_send_cam2pgrs(void)
{
	volatile unsigned int cam_gps_tmr;
/*
	static bool send_cam_now = false;
	
	if (!cam_gps_tmr) {
		cam_gps_tmr = 1000;
		send_cam_now =  (send_cam_now ? false:true);
	}

	if (send_cam_now)
		return true;

	return false;
	*/
	return true;
}

#include "lw_sd.h"
#include "lw_gprs.h"
#include "lw_gps.h"
#include "lw_vc0706.h"
#include "lw_nmea.h"

int main(void)
{
	uint32_t err_cnt = 0;
	uint32_t ok_cnt = 0;
	uint32_t ret;

	NVIC_Config();
	TIM2_Config();

	
#ifdef DEBUG
  debug();
#endif

#define TEST_USART
#define TEST_SPI

  /* Enable GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  
  /* Configure PC.06, PC.07, PC.08 and PC.09 as Output push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

#ifdef TEST_USART
	test_com_init();
#endif
#ifdef TEST_SPI
	test_spi_init();
#endif

#define TEST_GPRS
#ifdef TEST_GPRS
	lw_gprs_init();
	delay(150);
	lw_start_gprs_mode();
#endif

#define TEST_GPS
lw_gps_init();
#ifdef TEST_GPS
	lw_gps_init();
	lw_gps_param_init();
#endif

#define TEST_CAM
#ifdef TEST_CAM
	lw_cam2gprs_dma_init();
	lw_vc0706_init();
	lw_vc0706_param_init();	
#endif

  while (1)
  {
#ifdef TEST_USART
/*	test_com();
	continue;*/
#endif	
#ifdef TEST_SPI
/*	delay(2);
	test_spi();
	delay(2);
	continue;*/
#endif

//#define TEST_SD
#if (defined(TEST_SD) && defined(TEST_CAM) && !defined(TEST_GPRS) && !defined(TEST_GPS))
	
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


#ifdef TEST_SD
	while (1)
		//lw_sd_test();
	while(1) {
		lw_sd_with_fatfs_test();
		delay(100);
	}
/*	
com_send_message(1, "111");
	if(lw_sd_init() != SD_NO_ERR)
	{
		com_send_message(1, "sd_init err");
		delay(100);
		continue;
	}
	else
	{
		com_send_message(1, "sd_init ok");
		//delay(200);
	}
com_send_message(1, "222");	
	lw_sd_active();
	if(sd_info.card_type == CARDTYPE_SD)
		com_send_message(1, "card type: SD");
	else if(sd_info.card_type == CARDTYPE_MMC)
		com_send_message(1, "card type: MMC");
	delay(200);
	continue; */
#endif

#if (defined(TEST_GPRS) && !defined(TEST_GPS) && !defined(TEST_CAM))
	while(1) {test_gprs_1(); delay(20);}
#endif

#if (defined(TEST_GPS) && !defined(TEST_GPRS) && !defined(TEST_CAM))
	while(1) {lw_gps_test(); /*delay(5);*/}
#endif

#if (defined(TEST_GPS) && defined(TEST_GPRS) && !defined(TEST_CAM))
	while(1){
		if(( lw_gps_test()) == 0)
		{
			if((lw_nmea_test() ) == 0)
				lw_gprs_send_gps_test();
		}
	}
#endif

#if  (defined(TEST_CAM) && !defined(TEST_GPS) && !defined(TEST_GPRS))
	while(1){test_cam(); delay(30);}
#endif

#if 0 //(defined(TEST_GPRS) &&  defined(TEST_CAM) && !defined(TEST_GPS))
	while(1)
	{
		if(lw_cam_get_frame() == 0)
			lw_gprs_send_cam_frame();
		while(lw_cam_stop_frame() != 0 ) ;
		while(1);
			
	}
#endif

#if (defined(TEST_GPRS) &&  defined(TEST_CAM) && defined(TEST_GPS))
	while(1){
		extern volatile unsigned int Timer1, Timer2;
		Timer1 = 100;
		while(Timer1);
		//lw_cam_direct_to_gprs();
		if (( lw_gps_test() == 0) || (is_send_cam2pgrs()))
		{
			//if((lw_nmea_test() ) == 0)
			lw_nmea_test() ;
			lw_gprs_send_gps_test();

		}
	if(true == lw_gprs_isidle())
	{
		/*if(lw_cam_get_frame() == 0)
			lw_gprs_send_cam_frame();*/
		//while(lw_cam_stop_frame() != 0 ) ;
		//while(1);
			
	}
	}
#endif

    /* Turn on led connected to PC.06 pin */
    GPIO_SetBits(GPIOB, GPIO_Pin_12);

	delay(2);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);

	delay(2);
    /* Insert delay */
//    Delay(0xAFFFF);

    /* Turn on led connected to PC.07 and PC.08 pins */
//    GPIO_SetBits(GPIOB, GPIO_Pin_12 );
    /* Turn off led connected to PC.06 pin */
//    GPIO_ResetBits(GPIOC, GPIO_Pin_0);
    /* Insert delay */
//    Delay(0xAFFFF);

    /* Turn on led connected to PC.09 pin */
//    GPIO_SetBits(GPIOC, GPIO_Pin_9);
    /* Turn off led connected to PC.07 and PC.08 pins */
//    GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_7);
    /* Insert delay */
//    Delay(0xAFFFF);

    /* Turn off led connected to PC.09 pin */
//    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  }
}

