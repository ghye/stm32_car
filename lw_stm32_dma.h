#ifndef __LW_STM32_H__
#define __LW_STM32_H__

#include <stdint.h>
#include <stdbool.h>

void ais_rx_tx_dma_init(void);
void ais_rx_set_dma(void);
void ais_rx_reload_dma(void);
void ais_rx_dma_enable(void);
void ais_rx_dma_disable(void);
bool ais_rx_is_finished(void);
void ais_tx_set_dma(void);
void ais_tx_reload_dma(void);
void ais_tx_dma_enable(void);
void ais_tx_dma_disable(void);
bool ais_tx_is_finished(void);

#endif

