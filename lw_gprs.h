#ifndef __LW_GPRS_H__
#define __LW_GPRS_H__

#include <stdint.h>
#include <stdbool.h>

void lw_gprs_init(void);
void lw_start_gprs_mode(void);
void lw_seqed_msgs_init(void);

void lw_process_gprs_rbuf(uint8_t val);
void lw_get_gps_to_sbuf(void);
void lw_clean_seqed_msgs(void);
void lw_form_seqed_msgs(void);
int32_t lw_gprs_tcp_send_data(void);

#endif

