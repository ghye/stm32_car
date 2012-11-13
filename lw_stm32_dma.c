#include "projects_conf.h"

#if defined(STM_SHIP)

#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "lw_stm32_dma.h"
#include "lw_ais.h"

#if 0
#define AIS_RX_DMA_Channel	DMA1_Channel6 /*USART2_RX*/
#define AIS_RX_DMA_IRQ DMA1_Channel6_IRQn
#define AIS_RX_DMA_FLAG	DMA1_FLAG_TC6
#define AIS_RX_BASE  0x40004404	/*USART2_RX*/
#endif
#define AIS_RX_DMA_Channel	DMA1_Channel3 /*USART3_RX*/
#define AIS_RX_DMA_IRQ DMA1_Channel3_IRQn
#define AIS_RX_DMA_FLAG	DMA1_FLAG_TC3
#define AIS_RX_BASE  0x40004804	/*USART3_RX*/

#define AIS_TX_DMA_Channel	DMA1_Channel4 /*USART1_TX*/
#define AIS_TX_DMA_IRQ DMA1_Channel4_IRQn
#define AIS_TX_DMA_FLAG	DMA1_FLAG_TC4
#define AIS_TX_BASE  0x40013804	/*USART1_TX*/

DMA_InitTypeDef ais_rxDMA_InitStructure;
DMA_InitTypeDef ais_txDMA_InitStructure;

void ais_rx_tx_dma_init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void ais_rx_set_dma(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = AIS_RX_DMA_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//DMA_DeInit(AIS_RX_DMA_Channel);  
	DMA_Cmd(AIS_RX_DMA_Channel, DISABLE);
	DMA_SetCurrDataCounter(AIS_RX_DMA_Channel, AIS_RAW_MAX_LEN);
	ais_rxDMA_InitStructure.DMA_PeripheralBaseAddr = AIS_RX_BASE;
	ais_rxDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(ais_info.raw_info.raw[ais_info.raw_info.tail++]);
	if (ais_info.raw_info.tail >= AIS_RAW_MAX_NUM) 
		ais_info.raw_info.tail = 0;
	ais_rxDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	ais_rxDMA_InitStructure.DMA_BufferSize = AIS_RAW_MAX_LEN; /* for future */
	ais_rxDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	ais_rxDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	ais_rxDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	ais_rxDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	ais_rxDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	ais_rxDMA_InitStructure.DMA_Priority = DMA_Priority_High;
	ais_rxDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(AIS_RX_DMA_Channel, &ais_rxDMA_InitStructure);
	DMA_ITConfig(AIS_RX_DMA_Channel, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(AIS_RX_DMA_Channel, ENABLE);
	
}

void ais_rx_reload_dma(void)
{
	DMA_Cmd(AIS_RX_DMA_Channel, DISABLE);
	DMA_SetCurrDataCounter(AIS_RX_DMA_Channel, AIS_RAW_MAX_LEN);
	ais_rxDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(ais_info.raw_info.raw[ais_info.raw_info.tail++]);
	if (ais_info.raw_info.tail >= AIS_RAW_MAX_NUM) 
		ais_info.raw_info.tail = 0;
	DMA_Init(AIS_RX_DMA_Channel, &ais_rxDMA_InitStructure);
	DMA_Cmd(AIS_RX_DMA_Channel, ENABLE);
}

void ais_rx_dma_enable(void)
{
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(AIS_RX_DMA_Channel, ENABLE);
}

void ais_rx_dma_disable(void)
{
	USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);
	DMA_Cmd(AIS_RX_DMA_Channel, DISABLE);
}

bool ais_rx_is_finished(void)
{
	if(DMA_GetFlagStatus(AIS_RX_DMA_FLAG) == SET)
	{
		return true;
	}	
	return false;
}





void ais_tx_set_dma(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = AIS_TX_DMA_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//DMA_DeInit(AIS_RX_DMA_Channel);  
	DMA_Cmd(AIS_TX_DMA_Channel, DISABLE);
	DMA_SetCurrDataCounter(AIS_TX_DMA_Channel, ais_info.parsed_info.parsed_cnt);
	ais_txDMA_InitStructure.DMA_PeripheralBaseAddr = AIS_TX_BASE;
	ais_txDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ais_info.parsed_info.parsed;
	ais_txDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	ais_txDMA_InitStructure.DMA_BufferSize = ais_info.parsed_info.parsed_cnt; /* for future */
	ais_txDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	ais_txDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	ais_txDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	ais_txDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	ais_txDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	ais_txDMA_InitStructure.DMA_Priority = DMA_Priority_High;
	ais_txDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(AIS_TX_DMA_Channel, &ais_txDMA_InitStructure);
	DMA_ITConfig(AIS_TX_DMA_Channel, DMA_IT_TC, ENABLE);
	//USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	//DMA_Cmd(AIS_TX_DMA_Channel, ENABLE);
}

void ais_tx_reload_dma(void)
{
	DMA_Cmd(AIS_TX_DMA_Channel, DISABLE);
	DMA_SetCurrDataCounter(AIS_TX_DMA_Channel, ais_info.parsed_info.parsed_cnt);
	ais_txDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ais_info.parsed_info.parsed;
	ais_txDMA_InitStructure.DMA_BufferSize = ais_info.parsed_info.parsed_cnt;
	DMA_Init(AIS_TX_DMA_Channel, &ais_txDMA_InitStructure);
	DMA_Cmd(AIS_TX_DMA_Channel, ENABLE);
}

void ais_tx_dma_enable(void)
{
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(AIS_TX_DMA_Channel, ENABLE);
}

void ais_tx_dma_disable(void)
{
	USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
	DMA_Cmd(AIS_TX_DMA_Channel, DISABLE);
}

bool ais_tx_is_finished(void)
{
	if(DMA_GetFlagStatus(AIS_TX_DMA_FLAG) == SET)
	{
		return true;
	}	
	return false;
}

#endif

