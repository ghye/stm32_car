#ifndef __LOG_H__
#define __LOG_H__

#include "stdint.h"
#include "stdbool.h"

bool log_open_ack(void);
bool log_open_snd(void);
bool log_open_oth(void);
bool log_write_ack(uint8_t *buf, uint32_t len);
bool log_write_snd(uint8_t *buf, uint32_t len);
bool log_write_oth(uint8_t *buf, uint32_t len);

#endif

