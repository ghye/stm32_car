#ifndef __LOG_H__
#define __LOG_H__

#include "stdint.h"
#include "stdbool.h"

enum LOG_LEVEL {
	LOG_LEVEL_ERR = 0,
	LOG_LEVEL_WARN,
	LOG_LEVEL_MSG,
};

bool log_open(void);
bool log_close(void);
bool log_write(enum LOG_LEVEL level, uint8_t *buf, uint32_t len);

#endif

