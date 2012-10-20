#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "lw_stm32_dma.h"

#define CAM2GPRS_DMA_Channel	DMA1_Channel3
#define CAM2GPRS_DMA_FLAG	DMA1_FLAG_TC3
#define GPRS_TX_BASE 0x40013804
#define CAM_RX_BASE 0x40004804

void lw_cam2gprs_dma_init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void lw_cam2gprs_set_dma(uint32_t dma_size)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(CAM2GPRS_DMA_Channel);  
	DMA_InitStructure.DMA_PeripheralBaseAddr = GPRS_TX_BASE;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)CAM_RX_BASE;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = dma_size; /* for future */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(CAM2GPRS_DMA_Channel, &DMA_InitStructure);

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(CAM2GPRS_DMA_Channel, ENABLE);
	//USART_Cmd(USART3, ENABLE);
}

void lw_cam2gprs_dma_enable(void)
{
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(CAM2GPRS_DMA_Channel, ENABLE);
}

void lw_cam2gprs_dma_disable(void)
{
	USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);
	DMA_Cmd(CAM2GPRS_DMA_Channel, DISABLE);
}

int32_t lw_cam2gprs_is_finished(void)
{
	if(DMA_GetFlagStatus(CAM2GPRS_DMA_FLAG) == SET)
	{
		return 0;
	}	
	return -1;
}
