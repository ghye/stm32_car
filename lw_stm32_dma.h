#ifndef __LW_STM32_H__
#define __LW_STM32_H__

#include <stdint.h>
#include <stdbool.h>

void lw_cam2gprs_dma_init(void);
void lw_cam2gprs_set_dma(uint32_t dma_size);
void lw_cam2gprs_dma_enable(void);
void lw_cam2gprs_dma_disable(void);
int32_t lw_cam2gprs_is_finished(void);

#endif

