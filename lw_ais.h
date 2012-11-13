#ifndef __LW_AIS_H__
#define __LW_AIS_H__

#include "stdint.h"
#include "stdbool.h"

#define AIS_RAW_MAX_NUM 10
#define AIS_RAW_MAX_LEN 1024
#define AIS_PARSED_MAX_LEN (AIS_RAW_MAX_LEN<<1)
#define AIS_PARSED_TMP_MAX_LEN AIS_RAW_MAX_LEN
#define AIS_PARSED_LIST_MAX_LEN /*1 */(AIS_PARSED_MAX_LEN/80)

struct ais_info_ {
	struct raw_info_ {
		uint8_t tail;
		uint8_t raw[AIS_RAW_MAX_NUM][AIS_RAW_MAX_LEN];
	}raw_info;
	struct parsed_info_ {
		uint8_t head;
		uint32_t parsed_cnt;
		//uint16_t parsed_list[AIS_PARSED_LIST_MAX_LEN];
		uint8_t parsed[AIS_PARSED_MAX_LEN];
		uint32_t tmp_cnt;
		uint8_t tmp[AIS_PARSED_TMP_MAX_LEN];
	}parsed_info;
};
extern struct ais_info_ ais_info;

void ais_rx_init(void);
void ais_global_init(void);
void ais_parse_init(void);
void ais_seqed_proc(void);
#endif

