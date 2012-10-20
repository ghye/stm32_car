#ifndef __LW_GPRS_H__
#define __LW_GPRS_H__

#include <stdint.h>
#include <stdbool.h>

void lw_gprs_init(void);
void lw_start_gprs_mode(void);
void lw_process_gprs_rbuf(uint8_t val);
int32_t lw_gprs_send_gps_test(void);
void lw_gprs_send_C_cmd(double lat, double lon, double speed, 
	double track/*航向*/, bool isvalid /*数据状态: 1:有效，0:无效*/);
bool lw_gprs_isidle(void);
void test_gprs_1(void);


void lw_send_cam_cmd_L(uint32_t frame_len);
void lw_grps_send_cam_frame_len(uint32_t frame_len);


#endif
