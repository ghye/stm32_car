
#include <string.h>
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_spi.h"

#include "lw_stm32_spi.h"

#define Rx_Max 10
uint8_t SPI2_Buffer_Rx[Rx_Max];

#define SD_CS_HIGH() GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SD_CS_LOW() GPIO_ResetBits(GPIOA, GPIO_Pin_4)

void sd_cs_high(void)
{
	int8_t i;

	i = 10;
	while (i--)
		spi_send_char(0xFF);
	
	SD_CS_HIGH();

	spi_send_char(0xFF);
}

bool sd_cs_low(void)
{
	uint32_t i;

	i = 0;
	SD_CS_LOW();

	do {
		if (spi_recv_char() == 0xFF)
			return true;
		i++;
	}while(i < 0xffffff);

	return false;
}
void spi_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;	
	#define SPI2_DR_Address  0x4000380C	

	/* spi1->spi2 */

	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* GPIOA, GPIOB and SPI1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_SPI1, ENABLE);

	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	

	/* Configure SPI1 pins: NSS, SCK, MISO and MOSI ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

/*	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);	*/

	/* Configure SPI2 pins: NSS, SCK, MISO and MOSI ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* DMA1 channel4 configuration ---------------------------------------------*/
	DMA_DeInit(DMA1_Channel4);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI2_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) SPI2_Buffer_Rx;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = Rx_Max;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	/* SPI1 configuration ------------------------------------------------------*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	/* SPI2 configuration ------------------------------------------------------*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_Init(SPI2, &SPI_InitStructure);

	/* Enable SPI1 NSS output for master mode */
	SPI_SSOutputCmd(SPI1, ENABLE);

	/* Enable SPI2 Rx request */
//	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);	/* 将DMA关联到SPI2 */

	/* Enable SPI2 */
//	SPI_Cmd(SPI2, ENABLE);
	/* Enable SPI1 */
	SPI_Cmd(SPI1, ENABLE);

	/* Enable DMA1 Channel4 */
//	DMA_Cmd(DMA1_Channel4, ENABLE);


	SD_CS_HIGH();

}

void sd_spi_clk_tomax(void)
{
	SPI_InitTypeDef  SPI_InitStructure;	

	SPI_Cmd(SPI1, DISABLE);
	//SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);	
	SPI_Cmd(SPI1, ENABLE);
}

uint8_t spi_send_char(uint8_t val)
{
	/* Wait for SPI1 Tx buffer empty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	/* Send SPI1 data */
	SPI_I2S_SendData(SPI1, val);

	return 0;
}

uint8_t spi_send(uint8_t *sbuf, uint32_t length)
{
	uint32_t cnt=0;
	/* Transfer procedure */
	while (cnt < length)
	{
		spi_send_char(sbuf[cnt++]);
		#if 0
		/* Wait for SPI1 Tx buffer empty */
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		/* Send SPI1 data */
		SPI_I2S_SendData(SPI1, sbuf[cnt++]);
		#endif
	}	

	return 0;
}

uint8_t spi_recv_char(void)
{
	uint8_t val;

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) ;
	SPI_I2S_SendData(SPI1, 0xFF);
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) ;
	val = SPI_I2S_ReceiveData(SPI1);

	return val;
}
uint32_t spi_recv(uint8_t *rbuf)
{
	return 0;
	
#if 0
	/* Wait for DMA1 channel4 transfer complete */
	while (!DMA_GetFlagStatus(DMA1_FLAG_TC4));

	memcpy(rbuf, SPI2_Buffer_Rx, Rx_Max);

	return Rx_Max;
#endif	
}
