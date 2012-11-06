#include "ff.h"
#include "log.h"

#define logackname "gprs_ack.log"
#define logsndname "gprs_snd.log"
#define logothname "oth.log"

static FIL logackFil;            /* File object */
static FIL logsndFil;            /* File object */
static FIL logothFil;            /* File object */
static bool logackFlag = false;
static bool logsndFlag = false;
static bool logothFlag = false;

bool log_open(uint8_t num)
{
	switch (num) {
	case 1:
		if (f_open(&logackFil, logackname, FA_WRITE | FA_OPEN_ALWAYS)) {
			return false;
		}
		logackFlag = true;
		break;
	case 2:
		if (f_open(&logsndFil, logsndname, FA_WRITE | FA_OPEN_ALWAYS)) {
			return false;
		}
		logsndFlag = true;
		break;
	case 3:
		if (f_open(&logothFil, logothname, FA_WRITE | FA_OPEN_ALWAYS)) {
			return false;
		}
		logothFlag = true;
		break;
	}
	return true;
}

bool log_close(uint8_t num)
{
	switch (num) {
	case 1:
		if (f_close(&logackFil)) {
			return false;
		}
		logackFlag = false;
		break;
	case 2:
		if (f_close(&logsndFil)) {
			return false;
		}
		logsndFlag = false;
		break;
	case 3:
		if (f_close(&logothFil)) {
			return false;
		}
		logothFlag = false;
		break;
	}
	return true;
}

bool log_open_ack(void)
{
	return log_open(1);
}

bool log_open_snd(void)
{
	return log_open(2);
}

bool log_open_oth(void)
{
	return log_open(3);
}

bool log_close_ack(void)
{
	return log_close(1);
}

bool log_close_snd(void)
{
	return log_close(2);
}

bool log_close_oth(void)
{
	return log_close(3);
}

bool log_write_ack(uint8_t *buf, uint32_t len)
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
bool log_write_snd(uint8_t *buf, uint32_t len)
{
/*	uint32_t bw;
	
	if (!logsndFlag)
		return false;

	//f_lseek(&logsndFil, ~(uint64_t)0);
	if (f_write(&logsndFil, (void *) buf, len, &bw)) {
		log_close_snd();
		return false;
	}
	
	return true;*/
}
bool log_write_oth(uint8_t *buf, uint32_t len)
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


