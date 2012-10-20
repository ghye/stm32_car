#ifndef __LW_VC0706_H__
#define  __LW_VC0706_H__

#include <stdint.h>
#include <stdbool.h>

void lw_vc0706_init(void);
void lw_vc0706_param_init(void);
void test_cam(void);

int32_t lw_cam_get_frame(void);
int32_t lw_cam_start_frame(void);
int32_t lw_cam_stop_frame(void);
	
#endif

