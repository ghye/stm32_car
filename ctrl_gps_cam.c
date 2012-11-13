#include <stdint.h>
#include <stdbool.h>
#include "projects_conf.h"

struct _global_ctrl_info {
	bool iscam;
	bool isgps;
};
static struct _global_ctrl_info global_ctrl_info;

void ctrl_gps_cam_init(void)
{
	global_ctrl_info.iscam = false;
	global_ctrl_info.isgps = true;
}

void toggle_send_cam_gps(void)
{
	#define GPS_INT 3000
	#define CAM_INT 18000
	extern volatile unsigned int cam_tmr;
	extern volatile unsigned int gps_tmr;

#if defined(STM_CAR)
	if (!cam_tmr) {
		cam_tmr = CAM_INT;
		global_ctrl_info.iscam = true;
	}
#endif

	if (!gps_tmr) {
		gps_tmr = GPS_INT;
		global_ctrl_info.isgps = true;
	}
}
void ctrl_send_cam_or_gps_toggle(void)
{
#if 0
	#define GPS_INT 12000
	extern volatile unsigned int cam_gps_tmr;
	static uint16_t gpscnt = 0;

	if ((!cam_gps_tmr) && (!global_ctrl_info.iscam)) {
		gpscnt++;
		cam_gps_tmr = GPS_INT;
		global_ctrl_info.isgps = true;
	}
	if ((gpscnt >= 5) && (!global_ctrl_info.isgps)) {
		gpscnt = 0;
		global_ctrl_info.iscam = true;
	}
	
#elif 0
	#define CAM_INT 18000
	#define GPS_INT 6000
	
	extern volatile unsigned int cam_gps_tmr;
	static uint32_t gpstmp = 0;

	if (!cam_gps_tmr) {
		cam_gps_tmr = /*6000;*/CAM_INT;
		global_ctrl_info.iscam = true;
	}
	if (!(cam_gps_tmr/ /*1000 */GPS_INT %2) && (cam_gps_tmr/GPS_INT /*1000*/ != gpstmp)) {
		gpstmp = cam_gps_tmr/GPS_INT /*1000*/;
		global_ctrl_info.isgps = true;
	}
#endif
}

bool is_send_cam(void)
{
	return global_ctrl_info.iscam;
}

bool is_send_gps(void)
{
	return global_ctrl_info.isgps;
}

void set_no_send_cam(void)
{
	global_ctrl_info.iscam = false;
}

void set_no_send_gps(void)
{
	global_ctrl_info.isgps = false;
}


