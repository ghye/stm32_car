#ifndef __LW_GPRS_H__
#define __LW_GPRS_H__

#include <stdint.h>
#include <stdbool.h>

void lw_gprs_init(void);
void lw_start_gprs_mode(void);
void lw_seqed_msgs_init(void);

void gprs_send_cmd_f(uint8_t num, uint8_t *p, uint32_t len);
bool lw_get_seqed_msg(uint8_t *p);
void lw_process_gprs_rbuf(uint8_t val);
void lw_get_gps_to_sbuf(void);
void lw_clean_seqed_msgs(void);
void lw_form_seqed_msgs(void);
int32_t lw_gprs_tcp_send_data(void);
bool gprs_offline_20min(void);
bool cam_ready_used_save_sd(void);
bool get_cam_used_save_sd(uint8_t **buf, uint32_t *len);


#endif

