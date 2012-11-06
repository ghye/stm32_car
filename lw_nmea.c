#include <string.h>
#include <stdio.h>
#include "./ais/proto.h"
#include "lw_gps.h"
#include "lw_gprs.h"
//#include "nmea0813.h"

struct gps_device_t session;
int32_t lw_nmea_parse(void)
{
	int32_t nmea_ret;
	uint8_t *pgps;

	pgps = lw_get_gps_buf();
	session.packet.type = NMEA_PACKET;
	memcpy(session.packet.outbuffer, pgps, strlen(pgps));
	session.packet.outbuffer[strlen(pgps)] = '\0';
	session.packet.outbuflen = strlen(pgps);

	nmea_ret = aivdm_analyze(&session);

	if(nmea_ret)
	{
		switch(session.nmea.type)
		{
			case TYPE_GPRMC:
			lw_gprs_form_C_cmd(session.nmea.nmea_u.gprmc.lat, 
				session.nmea.nmea_u.gprmc.lon, session.nmea.nmea_u.gprmc.speed, 
				session.nmea.nmea_u.gprmc.track /*航向*/, 
				(session.nmea.nmea_u.gprmc.status==STATUS_FIX) ? 1:0 /*数据状态: 1:有效，0:无效*/);
			break;
			default:
			break;

		}

	}

	return 0;
}

