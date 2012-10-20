
#include <math.h>
#include <stdlib.h>
#include <string.h>
//#include "nmea0813.h"
#include "proto.h"

#define DD(s)   ((int32_t)((s)[0]-'0')*10+(int32_t)((s)[1]-'0'))

int32_t processGPRMC (int8_t count, int8_t *f[],
                       struct gps_device_t * session)
{
	double d, m;
	double lat;
	double lon;
	double mag;

	session->nmea.type = TYPE_GPRMC;
	session->nmea.nmea_u.gprmc.time.hour = DD(f[0]);
	session->nmea.nmea_u.gprmc.time.min = DD(&f[0][2]);
	session->nmea.nmea_u.gprmc.time.sec = DD(&f[0][4]);
	if(f[1][0] == 'V')
		session->nmea.nmea_u.gprmc.status = STATUS_NO_FIX;
	else if(f[1][0] == 'A')
		session->nmea.nmea_u.gprmc.status = STATUS_FIX;
/*	double d, m;
	double lat;*/
	lat = atof(f[2]);
#if 0
    m = 100.0 * modf(lat / 100.0, &d);
    lat = d + m / 60.0;

    if (f[3][0] == 'S')
        lat = -lat;
#endif	
	session->nmea.nmea_u.gprmc.lat = lat;

	//double lon;
    lon = atof(f[4]);
#if 0	
    m = 100.0 * modf(lon / 100.0, &d);
    lon = d + m / 60.0;
    if (f[5][0] == 'E')
        lon = -lon;
#endif
	session->nmea.nmea_u.gprmc.lon = lon;

	session->nmea.nmea_u.gprmc.speed = atof(f[6]);
	session->nmea.nmea_u.gprmc.track = atof(f[7]);

	session->nmea.nmea_u.gprmc.time.day = DD(&f[8][0]);
	session->nmea.nmea_u.gprmc.time.month = DD(&f[8][2]);
	session->nmea.nmea_u.gprmc.time.year = DD(&f[8][4]);

	//double mag;
	mag = atof(f[9]);
	if(f[10][0] == 'W')
		mag = -mag;
	session->nmea.nmea_u.gprmc.magnetic = mag;

	return true;
}

int32_t processGPGGA (int8_t count, int8_t *f[],
                       struct gps_device_t * session)
{
	double d, m;
	double lat;
	double lon;
	int8_t *p;

	session->nmea.type = TYPE_GPGGA;
	session->nmea.nmea_u.gpgga.time.hour = DD(f[0]);
	session->nmea.nmea_u.gpgga.time.min = DD(&f[0][2]);
	session->nmea.nmea_u.gpgga.time.sec = DD(&f[0][4]);

	/*double d, m;
	double lat;*/
	lat = atof(f[1]);
    m = 100.0 * modf(lat / 100.0, &d);
    lat = d + m / 60.0;
    if (f[2][0] == 'S')
        lat = -lat;
	session->nmea.nmea_u.gpgga.lat = lat;

	//double lon;
	lon = atof(f[3]);
    m = 100.0 * modf(lon / 100.0, &d);
    lon = d + m / 60.0;
    if (f[4][0] == 'S')
        lon = -lon;
	session->nmea.nmea_u.gpgga.lon = lon;

	switch(f[5][0]){
		case '0':
		session->nmea.nmea_u.gpgga.gps_quality = FIX_NOT_AVAIL;	
		break;

		case '1':
		session->nmea.nmea_u.gpgga.gps_quality = FIX;
		break;

		case '2':
		session->nmea.nmea_u.gpgga.gps_quality = DIFF_FIX;
		break;

		default:
		session->nmea.nmea_u.gpgga.gps_quality = FIX_UNDEF;	/* unknow */
		break;
	}

	session->nmea.nmea_u.gpgga.nsv = atoi(f[6]);
	session->nmea.nmea_u.gpgga.hdp = atof(f[7]);
	session->nmea.nmea_u.gpgga.antenna = atof(f[8]);
	session->nmea.nmea_u.gpgga.geoidal = atof(f[10]);
	session->nmea.nmea_u.gpgga.age = atof(f[12]);

	//int8_t *p;
	p = f[13];
	while(*p != '*')
		p++;
	*p = '\0';
	session->nmea.nmea_u.gpgga.stationID = atof(f[13]);

	return true;
}

int32_t processGPVTG (int8_t count, int8_t *f[],
                       struct gps_device_t * session)
{
	session->nmea.type = TYPE_GPVTG;
	session->nmea.nmea_u.gpvtg.t_track = atof(f[0]);
	session->nmea.nmea_u.gpvtg.m_track = atof(f[2]);
	session->nmea.nmea_u.gpvtg.knots_speed = atof(f[4]);
	session->nmea.nmea_u.gpvtg.killmeters_speed = atof(f[6]);

	return true;
}

int32_t nmea_parse(int8_t *buf, struct gps_device_t *session)
{
    typedef int32_t (*nmea_decoder) (int8_t count, int8_t *f[],
                       struct gps_device_t * session);
    static struct
    {        
    int8_t *name;
    int32_t nf;         /* minimum number of fields required to parse */
    /*bool cycle_continue;*/    /* cycle continuer? */
    nmea_decoder decoder;
    } nmea_phrase[] = {
		{"GPRMC", 12, processGPRMC},
		{"GPGGA", 15, processGPGGA},
		{"GPVTG", 15, processGPVTG},
	};
	int8_t *p;
	int8_t count = 0;
	int32_t i;
	int32_t j;
	int32_t retval;

	memset(&session->nmea, 0, sizeof(session->nmea));

	/*int8_t *p;
	int8_t count = 0;*/
	p = session->nmea.buf;
	strcpy(p, buf);
	p[strlen(buf)] = '\0';
	while(*p != '\0')
	{
		if(*p == ',')
		{
			*p = '\0';
			count++;
		}
		p++;
	}

	//int32_t i;
	for(i=0; i<sizeof(nmea_phrase)/sizeof(nmea_phrase[0]); i++)
		if(strcmp(nmea_phrase[i].name, session->nmea.buf+1) == 0)
		{ break;
		}
	if(i == sizeof(nmea_phrase)/sizeof(nmea_phrase[0]))
		return false;	/* unknow decoder */

	//int32_t j;
	p = session->nmea.buf;
	for(j=1; j<=count; j++)
	{
		session->nmea.filed[j-1] = p + strlen(p) +1;
		p += strlen(p) + 1;
	}
	session->nmea.filedlen = j;
	if(nmea_phrase[i].nf < count)
		return false;

	retval = (nmea_phrase[i].decoder)(session->nmea.filedlen, session->nmea.filed, session);
//int32_t processGPRMC (int8_t count, int8_t *f[],
 //                      struct gps_device_t * session)

	return retval;
}
