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
void RCC_Configuration(void);
void NVIC_Configuration(void);
void Delay(vu32 nCount);

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
	RCC_DeInit ();                        /* RCC system reset(for debug purpose)*/
	RCC_HSEConfig (RCC_HSE_ON);           /* Enable HSE */

	/* Wait till HSE is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);

	RCC_HCLKConfig   (RCC_SYSCLK_Div1);   /* HCLK   = SYSCLK  */
	RCC_PCLK2Config  (RCC_HCLK_Div1);     /* PCLK2  = HCLK    */
	RCC_PCLK1Config  (RCC_HCLK_Div2);     /* PCLK1  = HCLK/2  */
	RCC_ADCCLKConfig (RCC_PCLK2_Div6);    /* ADCCLK = PCLK2/6 */

	FLASH_SetLatency(FLASH_Latency_2);    /* Flash 2 wait state */
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

	/* PLLCLK = 8MHz * 9 = 72 MHz */
	RCC_PLLConfig (RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

	RCC_PLLCmd (ENABLE);                  /* Enable PLL */

	/* Wait till PLL is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

	/* Select PLL as system clock source */
	RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source */
	while (RCC_GetSYSCLKSource() != 0x08);
}

void NVIC_Configuration(void)
{ 
#ifdef  VECT_TAB_RAM  
	/* Set the Vector Table base location at 0x20000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


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

#include "lw_sd.h"
#include "lw_gprs.h"
#include "lw_gps.h"
#include "lw_vc0706.h"
#include "lw_nmea.h"

int main(void)
{
	//RCC_Configuration();
	//NVIC_Configuration();
	
#ifdef DEBUG
  debug();
#endif

#define TEST_USART
#define TEST_SPI

  /* Configure the system clocks */
//  RCC_Configuration();
    
  /* NVIC Configuration */
//  NVIC_Configuration();

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
//lw_gps_init();
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

#define TEST_SD
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

#if (defined(TEST_GPRS) &&  defined(TEST_CAM) && !defined(TEST_GPS))
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
		if(( lw_gps_test()) == 0)
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

