#ifndef __CTRL_GPS_CAM_H__
#define __CTRL_GPS_CAM_H__

#include <stdbool.h>

void ctrl_gps_cam_init(void);
void ctrl_send_cam_or_gps_toggle(void);
bool is_send_cam(void);
bool is_send_gps(void);
void set_no_send_cam(void);
void set_no_send_gps(void);

#endif


