
#ifndef __LW_STM32_SPI_H__
#define __LW_STM32_SPI_H__

//#include <stm32f10x_type.h>
//#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>

void sd_cs_high(void);
bool sd_cs_low(void);
void spi_init(void);
void sd_spi_clk_tomax(void);
uint8_t spi_send_char(uint8_t val);
uint8_t spi_send(uint8_t *sbuf, uint32_t length);
uint8_t spi_recv_char(void);
uint32_t spi_recv(uint8_t *rbuf);

#endif

