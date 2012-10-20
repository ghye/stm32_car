#include <string.h>
#include <stdio.h>
#include "proto.h"
//#include "nmea0813.h"


void test_nmea(void)
{
	//char buf[]="$GPRMC,225446.33,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E,A*68";
	//char buf[]="$GPGGA,094547.00,2305.79852,N,11326.69546,E,2,12,0.94,27.2,M,-5.0,M,,*7F";
	char buf[]="$GPVTG,86.03,T,,M,0.000,N,0.000,K,D*05";
	int buflen = strlen(buf);
	struct gps_device_t session;
	
	session.packet.type = NMEA_PACKET;
	memcpy(session.packet.outbuffer, buf, buflen);
	session.packet.outbuffer[buflen] = '\0';
	session.packet.outbuflen = buflen;

	int ret = aivdm_analyze(&session);

	if(ret)
	{
		switch(session.nmea.type)
		{
			case TYPE_GPRMC:
			printf("GPRMC:\n");
			printf("status:%d\n", session.nmea.nmea_u.gprmc.status);
			printf("time:%d:%d:%d:%d:%d:%d\n", session.nmea.nmea_u.gprmc.time.year,
												session.nmea.nmea_u.gprmc.time.month,
												session.nmea.nmea_u.gprmc.time.day,
												session.nmea.nmea_u.gprmc.time.hour,
												session.nmea.nmea_u.gprmc.time.min,
												session.nmea.nmea_u.gprmc.time.sec);
			printf("lat:%f\n", session.nmea.nmea_u.gprmc.lat);
			printf("lon:%f\n", session.nmea.nmea_u.gprmc.lon);
			printf("speed:%f\n", session.nmea.nmea_u.gprmc.speed);
			printf("track:%f\n", session.nmea.nmea_u.gprmc.track);
			printf("magnetic:%f\n", session.nmea.nmea_u.gprmc.magnetic);
			break;
			case TYPE_GPGGA:
			printf("GPGGA:\n");
			printf("time:%d:%d:%d\n", session.nmea.nmea_u.gpgga.time.hour,
												session.nmea.nmea_u.gpgga.time.min,
												session.nmea.nmea_u.gpgga.time.sec);
			printf("gps_quality:%d\n", session.nmea.nmea_u.gpgga.gps_quality);
			printf("nsv:%d\n", session.nmea.nmea_u.gpgga.nsv);
			printf("hdp:%f\n", session.nmea.nmea_u.gpgga.hdp);
			printf("antenna:%f\n", session.nmea.nmea_u.gpgga.antenna);
			printf("geoidal:%f\n", session.nmea.nmea_u.gpgga.geoidal);
			printf("age:%f\n", session.nmea.nmea_u.gpgga.age);
			printf("statusID:%d\n", session.nmea.nmea_u.gpgga.stationID);
			break;
			case TYPE_GPVTG:
			printf("GPVTG:\n");
			printf("t_track:%f\n", session.nmea.nmea_u.gpvtg.t_track);
			printf("m_track:%f\n", session.nmea.nmea_u.gpvtg.m_track);
			printf("knots_speed:%f\n", session.nmea.nmea_u.gpvtg.knots_speed);
			printf("killmeters_speed:%f\n", session.nmea.nmea_u.gpvtg.killmeters_speed);
			break;
			default:
			break;
		}
	}
}

void test_aivdm(void)
{

#define test_type1_2_3 0
#define test_type5 1

	//unsigned char buf[]="!AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C";
	unsigned char buf[]="!AIVDM,1,1,,B,168upK000087BN4==h;h03CJ25:l,0*03";
	//unsigned char buf5[]="!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E";
	//unsigned char buf5[]="!AIVDM,1,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E531@0000000000000,2*3E";
	unsigned char buf5[]="!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E";
	unsigned char buf5_2[]="!AIVDM,2,2,3,B,1@0000000000000,2*55";
	struct gps_device_t session;

	memset(&session, 0, sizeof(session));
	session.packet.type = AIVDM_PACKET;
#if (test_type5)
	int i=2;
	while(i-->0)
	{
	if(i==1)
	{
		memcpy(session.packet.outbuffer, buf5, strlen((char *)buf5));/*待解析的可见字符串*/
		session.packet.outbuflen = strlen((char *)buf5);/*长度*/
	}
	else
	{
		memcpy(session.packet.outbuffer, buf5_2, strlen((char *)buf5_2));/*待解析的可见字符串*/
		session.packet.outbuflen = strlen((char *)buf5_2);/*长度*/
	}
#elif (test_type1_2_3)
	memcpy(session.packet.outbuffer, buf, strlen((char *)buf));/*待解析的可见字符串*/
	session.packet.outbuflen = strlen((char *)buf);/*长度*/
#endif
	
	int ret = aivdm_analyze(&session);

	/*查看*/
	printf("ais type:%d\n", session.gpsdata.ais.type);
	if(ret & AIS_SET)
	switch (session.gpsdata.ais.type)
	{
		case 1:
		printf("ais UTC:%d\n", session.gpsdata.ais.type1.second);
		printf("ais MMSI:%d\n", session.gpsdata.ais.mmsi);
		printf("ais Latitude:%d,o:%d\n", session.gpsdata.ais.type1.lat /600000, session.gpsdata.ais.type1.lat);
		printf("ais Longitude:%d,o:%d\n", session.gpsdata.ais.type1.lon /600000, session.gpsdata.ais.type1.lon);
		printf("ais Speed:%d\n", session.gpsdata.ais.type1.speed /10);
		printf("ais Heading:%d\n", session.gpsdata.ais.type1.heading);
		printf("ais Course over ground:%f\n", ((double)(session.gpsdata.ais.type1.course)) /10 +\
												(session.gpsdata.ais.type1.course &0x01) * (float)0.1 );
		printf("ais Rate of turn:%d\n", session.gpsdata.ais.type1.turn);
		printf("ais Navigational status:%d\n", session.gpsdata.ais.type1.status);
		/*printf("ais Nearest place:%d\n", session.gpsdata.type1.);*/
		break;
	
		case 2:
		printf("ais UTC:%d\n", session.gpsdata.ais.type1.second);
		printf("ais MMSI:%d\n", session.gpsdata.ais.mmsi);
		printf("ais Latitude:%d,o:%d\n", session.gpsdata.ais.type1.lat /600000, session.gpsdata.ais.type1.lat);
		printf("ais Longitude:%d,o:%d\n", session.gpsdata.ais.type1.lon /600000, session.gpsdata.ais.type1.lon);
		printf("ais Speed:%d\n", session.gpsdata.ais.type1.speed /10);
		printf("ais Heading:%d\n", session.gpsdata.ais.type1.heading);
		printf("ais Course over ground:%f\n", ((double)(session.gpsdata.ais.type1.course)) /10 +\
												(session.gpsdata.ais.type1.course &0x01) * (float)0.1 );
		printf("ais Rate of turn:%d\n", session.gpsdata.ais.type1.turn);
		printf("ais Navigational status:%d\n", session.gpsdata.ais.type1.status);
		/*printf("ais Nearest place:%d\n", session.gpsdata.type1.);*/
		break;

		case 3:
		printf("ais UTC:%d\n", session.gpsdata.ais.type1.second);
		printf("ais MMSI:%d\n", session.gpsdata.ais.mmsi);
		printf("ais Latitude:%d,o:%d\n", session.gpsdata.ais.type1.lat /600000, session.gpsdata.ais.type1.lat);
		printf("ais Longitude:%d,o:%d\n", session.gpsdata.ais.type1.lon /600000, session.gpsdata.ais.type1.lon);
		printf("ais Speed:%d\n", session.gpsdata.ais.type1.speed /10);
		printf("ais Heading:%d\n", session.gpsdata.ais.type1.heading);
		printf("ais Course over ground:%f\n", ((double)(session.gpsdata.ais.type1.course)) /10 +\
												(session.gpsdata.ais.type1.course &0x01) * (float)0.1 );
		printf("ais Rate of turn:%d\n", session.gpsdata.ais.type1.turn);
		printf("ais Navigational status:%d\n", session.gpsdata.ais.type1.status);
		/*printf("ais Nearest place:%d\n", session.gpsdata.type1.);*/
		break;

		case 5:
		printf("ais version:%d\n", session.gpsdata.ais.type5.ais_version);
		printf("ais MMSI:%d\n", session.gpsdata.ais.mmsi);
		printf("ais imo:%d\n", session.gpsdata.ais.type5.imo);
		printf("ais shiptype:%d\n", session.gpsdata.ais.type5.shiptype);
		printf("ais to_bow:%d\n", session.gpsdata.ais.type5.to_bow);
		printf("ais to_stern:%d\n", session.gpsdata.ais.type5.to_stern);
		printf("ais to_port:%d\n", session.gpsdata.ais.type5.to_port);
		printf("ais to_starboard:%d\n", session.gpsdata.ais.type5.to_starboard);
		printf("ais epfd:%d\n", session.gpsdata.ais.type5.epfd);
		printf("ais time:%d:%d:%d:%d\n", session.gpsdata.ais.type5.month,session.gpsdata.ais.type5.day,
										session.gpsdata.ais.type5.hour,session.gpsdata.ais.type5.minute);
		printf("ais draught:%d\n", session.gpsdata.ais.type5.draught);
		printf("ais dte:%d\n", session.gpsdata.ais.type5.dte);
		printf("ais callsign:%s\n", session.gpsdata.ais.type5.callsign);
		printf("ais shipname:%s\n", session.gpsdata.ais.type5.shipname);
		printf("ais destination:%s\n", session.gpsdata.ais.type5.destination);
		break;
	}
#if (test_type5)
}
#endif
}

int main(void)
{
	test_aivdm();
	
	test_nmea();

	return 0;
}

