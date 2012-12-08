#include "ff.h"
#include "log.h"

#define logname "log.log"

static FIL logFil;            /* File object */
static bool logFlag = false;

#define CACHE_SIZE 512
#define NEW_LINE "\x00D\x00A"
static uint8_t cache[CACHE_SIZE];
static uint16_t cache_index = 0;

bool log_open(void)
{
	if (f_open(&logFil, logname, FA_WRITE | FA_OPEN_ALWAYS)) {
		return false;
	}
	logFlag = true;
	return true;
}

bool log_close(void)
{
	if (f_close(&logFil)) {
		return false;
	}
	logFlag = false;
	return true;
}

bool log_write(enum LOG_LEVEL level, uint8_t *buf, uint32_t len)
{
/*	uint32_t bw;
	
	if (!logackFlag)
		return false;

	//f_lseek(&logackFil, ~(uint64_t)0);
	if (f_write(&logackFil, (void *) buf, len, &bw)) {
		log_close_ack();
		return false;
	}
	
	return true;*/
}


