#include "lw_gprs_send_proc.h"

#include "lw_gprs.h"
#include "ctrl_gps_cam.h"

int32_t lw_gprs_send_data(void)
{
	debug_printf_m("start lw_gprs_send_data");
	lw_get_gps_to_sbuf();
	lw_clean_seqed_msgs();
	lw_form_seqed_msgs();
	toggle_send_cam_gps();
	lw_gprs_tcp_send_data();
	debug_printf_m("end lw_gprs_send_data");
	
	return 0;
}

