#ifndef __PROTO_H__
#define __PROTO_H__

#include <stdint.h>
//#include "nmea0813.h"
#include "common.h"

#define AIVDM_CHANNELS 2 /*A, B*/

/*   
 * For NMEA-conforming receivers this is supposed to be 82, but
 * some receivers (TN-200, GSW 2.3.2) emit oversized sentences.
 * The current hog champion is the Trimble BX-960 receiver, which
 * emits a 91-character GGA message.
 */     
#define NMEA_MAX    91      /* max length of NMEA sentence */
#define NMEA_BIG_BUF    (2*NMEA_MAX+1)  /* longer than longest NMEA sentence */
    


/* logging levels */
#define LOG_ERROR 	-1	/* errors, display always */
#define LOG_SHOUT	0	/* not an error but we should always see it */
#define LOG_WARN	1	/* not errors but may indicate a problem */
#define LOG_INF 	2	/* key informative messages */
#define LOG_DATA	3	/* log data management messages */
#define LOG_PROG	4	/* progress messages */
#define LOG_IO  	5	/* IO to and from devices */
#define LOG_SPIN	6	/* logging for catching spin bugs */
#define LOG_RAW 	7	/* raw low-level I/O */


/*
 * The packet buffers need to be as long than the longest packet we
 * expect to see in any protocol, because we have to be able to hold
 * an entire packet for checksumming...
 * First we thought it had to be big enough for a SiRF Measured Tracker
 * Data packet (188 bytes). Then it had to be big enough for a UBX SVINFO
 * packet (206 bytes). Now it turns out that a couple of ITALK messages are
 * over 512 bytes. I know we like verbose output, but this is ridiculous.
 */ 
#define MAX_PACKET_LENGTH   516 /* 7 + 506 + 3 */


struct gps_packet_t {
    /* packet-getter internals */
    int32_t	type;
#define BAD_PACKET      	-1
#define COMMENT_PACKET  	0
#define NMEA_PACKET     	1
#define AIVDM_PACKET    	2
#define GARMINTXT_PACKET	3
#define MAX_TEXTUAL_TYPE	3	/* increment this as necessary */
#define SIRF_PACKET     	4
#define ZODIAC_PACKET   	5
#define TSIP_PACKET     	6
#define EVERMORE_PACKET 	7
#define ITALK_PACKET    	8
#define GARMIN_PACKET   	9
#define NAVCOM_PACKET   	10
#define UBX_PACKET      	11
#define SUPERSTAR2_PACKET	12
#define ONCORE_PACKET   	13
#define GEOSTAR_PACKET   	14
#define NMEA2000_PACKET 	15
#define MAX_GPSPACKET_TYPE	15	/* increment this as necessary */
#define RTCM2_PACKET    	16
#define RTCM3_PACKET    	17
#define JSON_PACKET    	    	18
#define TEXTUAL_PACKET_TYPE(n)	((((n)>=NMEA_PACKET) && ((n)<=MAX_TEXTUAL_TYPE)) || (n)==JSON_PACKET)
#define GPS_PACKET_TYPE(n)	(((n)>=NMEA_PACKET) && ((n)<=MAX_GPSPACKET_TYPE))
#define LOSSLESS_PACKET_TYPE(n)	(((n)>=RTCM2_PACKET) && ((n)<=RTCM3_PACKET))
#define PACKET_TYPEMASK(n)	(1 << (n))
#define GPS_TYPEMASK	(((2<<(MAX_GPSPACKET_TYPE+1))-1) &~ PACKET_TYPEMASK(COMMENT_PACKET))
    uint32_t state;
    size_t length;
    uint8_t inbuffer[MAX_PACKET_LENGTH*2+1];
    size_t inbuflen;
    /*@observer@*/uint8_t *inbufptr;
    /* outbuffer needs to be able to hold 4 GPGSV records at once */
    uint8_t outbuffer[MAX_PACKET_LENGTH*2+1];
    size_t outbuflen;
    ulong_t char_counter;		/* count characters processed */
    ulong_t retry_counter;	/* count sniff retries */
    uint32_t counter;			/* packets since last driver switch */
    int32_t debug;				/* lexer debug level */
#ifdef TIMING_ENABLE
    timestamp_t start_time;		/* timestamp of first input */
    ulong_t start_char;		/* char counter at first input */
#endif /* TIMING_ENABLE */
    /*
     * ISGPS200 decoding context.
     *
     * This is not conditionalized on RTCM104_ENABLE because we need to
     * be able to build gpsdecode even when RTCM support is not
     * configured in the daemon.
     */
    struct {
	bool            locked;
	int32_t             curr_offset;
	/*isgps30bits_t   curr_word;*/
	uint32_t    bufindex;
	/*
	 * Only these should be referenced from elsewhere, and only when
	 * RTCM_MESSAGE has just been returned.
	 */
	/*isgps30bits_t   buf[RTCM2_WORDS_MAX];*/   /* packet data */
	size_t          buflen;                 /* packet length in bytes */
    } isgps;
#ifdef PASSTHROUGH_ENABLE
    uint32_t json_depth;
    uint32_t json_after;
#endif /* PASSTHROUGH_ENABLE */
};


/* state for resolving AIVDM decodes */
struct aivdm_context_t {
    /* hold context for decoding AIDVM packet sequences */
    int32_t decoded_frags;		/* for tracking AIDVM parts in a multipart sequence */
    uint8_t bits[2048];
    size_t bitlen; /* how many valid bits */
    /*struct ais_type24_queue_t type24_queue;*/
};

struct ais_t
{
    uint32_t	type;		/* message type */
    uint32_t    	repeat;		/* Repeat indicator */
    uint32_t	mmsi;		/* MMSI */
    union {
	/* Types 1-3 Common navigation info */
	struct {
	    uint32_t status;		/* navigation status */
	    int32_t turn;			/* rate of turn */
#define AIS_TURN_HARD_LEFT	-127
#define AIS_TURN_HARD_RIGHT	127
#define AIS_TURN_NOT_AVAILABLE	128
	    uint32_t speed;			/* speed over ground in deciknots */
#define AIS_SPEED_NOT_AVAILABLE	1023
#define AIS_SPEED_FAST_MOVER	1022		/* >= 102.2 knots */
	    bool accuracy;			/* position accuracy */
#define AIS_LATLON_DIV	600000.0
	    int32_t lon;				/* longitude */
#define AIS_LON_NOT_AVAILABLE	0x6791AC0
	    int32_t lat;				/* latitude */
#define AIS_LAT_NOT_AVAILABLE	0x3412140
	    uint32_t course;		/* course over ground */
#define AIS_COURSE_NOT_AVAILABLE	3600
	    uint32_t heading;		/* true heading */
#define AIS_HEADING_NOT_AVAILABLE	511
	    uint32_t second;		/* seconds of UTC timestamp */
#define AIS_SEC_NOT_AVAILABLE	60
#define AIS_SEC_MANUAL		61
#define AIS_SEC_ESTIMATED	62
#define AIS_SEC_INOPERATIVE	63
	    uint32_t maneuver;	/* maneuver indicator */
	    //uint32_t spare;	spare bits */
	    bool raim;			/* RAIM flag */
	    uint32_t radio;		/* radio status bits */
	} type1;
	/* Type 4 - Base Station Report & Type 11 - UTC and Date Response */
	struct {
	    uint32_t year;			/* UTC year */
#define AIS_YEAR_NOT_AVAILABLE	0
	    uint32_t month;			/* UTC month */
#define AIS_MONTH_NOT_AVAILABLE	0
	    uint32_t day;			/* UTC day */
#define AIS_DAY_NOT_AVAILABLE	0
	    uint32_t hour;			/* UTC hour */
#define AIS_HOUR_NOT_AVAILABLE	24
	    uint32_t minute;		/* UTC minute */
#define AIS_MINUTE_NOT_AVAILABLE	60
	    uint32_t second;		/* UTC second */
#define AIS_SECOND_NOT_AVAILABLE	60
	    bool accuracy;		/* fix quality */
	    int32_t lon;			/* longitude */
	    int32_t lat;			/* latitude */
	    uint32_t epfd;		/* type of position fix device */
	    //uint32_t spare;	spare bits */
	    bool raim;			/* RAIM flag */
	    uint32_t radio;		/* radio status bits */
	} type4;
	/* Type 5 - Ship static and voyage related data */
	struct {
	    uint32_t ais_version;	/* AIS version level */
	    uint32_t imo;		/* IMO identification */
	    int8_t callsign[7+1];		/* callsign */ 
#define AIS_SHIPNAME_MAXLEN	20
	    int8_t shipname[AIS_SHIPNAME_MAXLEN+1];	/* vessel name */
	    uint32_t shiptype;	/* ship type code */
	    uint32_t to_bow;	/* dimension to bow */
	    uint32_t to_stern;	/* dimension to stern */
	    uint32_t to_port;	/* dimension to port */
	    uint32_t to_starboard;	/* dimension to starboard */
	    uint32_t epfd;		/* type of position fix deviuce */
	    uint32_t month;		/* UTC month */
	    uint32_t day;		/* UTC day */
	    uint32_t hour;		/* UTC hour */
	    uint32_t minute;	/* UTC minute */
	    uint32_t draught;	/* draft in meters */
	    int8_t destination[20+1];	/* ship destination */
	    uint32_t dte;		/* data terminal enable */
	    //uint32_t spare;	spare bits */
	} type5;
	/* Type 6 - Addressed Binary Message */
	struct {
	    uint32_t seqno;		/* sequence number */
	    uint32_t dest_mmsi;	/* destination MMSI */
	    bool retransmit;		/* retransmit flag */
	    //uint32_t spare;	spare bit(s) */
	    uint32_t dac;           /* Application ID */
	    uint32_t fid;           /* Functional ID */
#define AIS_TYPE6_BINARY_MAX	920	/* 920 bits */
	    size_t bitcount;		/* bit count of the data */
	    union {
		int8_t bitdata[(AIS_TYPE6_BINARY_MAX + 7) / 8];
		/* GLA - AtoN monitoring data (UK/ROI) */
		struct {
		    uint32_t ana_int;       /* Analogue (internal) */
		    uint32_t ana_ext1;      /* Analogue (external #1) */
		    uint32_t ana_ext2;      /* Analogue (external #2) */
		    uint32_t racon; /* RACON status */
		    uint32_t light; /* Light status */
		    bool alarm; /* Health alarm*/
		    uint32_t stat_ext;      /* Status bits (external) */
		    bool off_pos;    /* Off position status */
		} dac235fid10;
		/* IMO236 - Dangerous Cargo Indication */
		struct {
		    int8_t lastport[5+1];		/* Last Port Of Call */
		    uint32_t lmonth;	/* ETA month */
		    uint32_t lday;		/* ETA day */
		    uint32_t lhour;		/* ETA hour */
		    uint32_t lminute;	/* ETA minute */
		    int8_t nextport[5+1];		/* Next Port Of Call */
		    uint32_t nmonth;	/* ETA month */
		    uint32_t nday;		/* ETA day */
		    uint32_t nhour;		/* ETA hour */
		    uint32_t nminute;	/* ETA minute */
		    int8_t dangerous[20+1];	/* Main Dangerous Good */
		    int8_t imdcat[4+1];		/* IMD Category */
		    uint32_t unid;		/* UN Number */
		    uint32_t amount;	/* Amount of Cargo */
		    uint32_t unit;		/* Unit of Quantity */
		} dac1fid12;
		/* IMO236 - Extended Ship Static and Voyage Related Data */
		struct {
		    uint32_t airdraught;	/* Air Draught */
		} dac1fid15;
		/* IMO236 - Number of Persons on board */
		struct {
		    uint32_t persons;	/* number of persons */
		} dac1fid16;
		/* IMO289 - Clearance Time To Enter Port */
		struct {
		    uint32_t linkage;	/* Message Linkage ID */
		    uint32_t month;	/* Month (UTC) */
		    uint32_t day;	/* Day (UTC) */
		    uint32_t hour;	/* Hour (UTC) */
		    uint32_t minute;	/* Minute (UTC) */
		    int8_t portname[20+1];	/* Name of Port & Berth */
		    int8_t destination[5+1];	/* Destination */
		    int32_t lon;	/* Longitude */
		    int32_t lat;	/* Latitude */
		} dac1fid18;
		/* IMO289 - Berthing Data (addressed) */
		struct {
		    uint32_t linkage;	/* Message Linkage ID */
		    uint32_t berth_length;	/* Berth length */
		    uint32_t berth_depth;	/* Berth Water Depth */
		    uint32_t position;	/* Mooring Position */
		    uint32_t month;	/* Month (UTC) */
		    uint32_t day;	/* Day (UTC) */
		    uint32_t hour;	/* Hour (UTC) */
		    uint32_t minute;	/* Minute (UTC) */
		    uint32_t availability;	/* Services Availability */
		    uint32_t agent;	/* Agent */
		    uint32_t fuel;	/* Bunker/fuel */
		    uint32_t chandler;	/* Chandler */
		    uint32_t stevedore;	/* Stevedore */
		    uint32_t electrical;	/* Electrical */
		    uint32_t water;	/* Potable water */
		    uint32_t customs;	/* Customs house */
		    uint32_t cartage;	/* Cartage */
		    uint32_t crane;	/* Crane(s) */
		    uint32_t lift;	/* Lift(s) */
		    uint32_t medical;	/* Medical facilities */
		    uint32_t navrepair;	/* Navigation repair */
		    uint32_t provisions;	/* Provisions */
		    uint32_t shiprepair;	/* Ship repair */
		    uint32_t surveyor;	/* Surveyor */
		    uint32_t steam;	/* Steam */
		    uint32_t tugs;	/* Tugs */
		    uint32_t solidwaste;	/* Waste disposal (solid) */
		    uint32_t liquidwaste;	/* Waste disposal (liquid) */
		    uint32_t hazardouswaste;	/* Waste disposal (hazardous) */
		    uint32_t ballast;	/* Reserved ballast exchange */
		    uint32_t additional;	/* Additional services */
		    uint32_t regional1;	/* Regional reserved 1 */
		    uint32_t regional2;	/* Regional reserved 2 */
		    uint32_t future1;	/* Reserved for future */
		    uint32_t future2;	/* Reserved for future */
		    int8_t berth_name[20+1];	/* Name of Berth */
		    int32_t berth_lon;	/* Longitude */
		    int32_t berth_lat;	/* Latitude */
		} dac1fid20;
		/* IMO289 - Weather observation report from ship */
		/*** WORK IN PROGRESS - NOT YET DECODED ***/
		struct {
		    bool wmo;			/* true if WMO variant */
		    union {
			struct {
			    int8_t location[20+1];	/* Location */
			    int32_t lon;		/* Longitude */
			    int32_t lat;		/* Latitude */
			    uint32_t day;		/* Report day */
			    uint32_t hour;		/* Report hour */
			    uint32_t minute;	/* Report minute */
			    bool vislimit;		/* Max range? */
			    uint32_t visibility;	/* Units of 0.1 nm */
#define DAC8FID21_VISIBILITY_NOT_AVAILABLE	127
#define DAC8FID21_VISIBILITY_SCALE		10.0
			    uint32_t humidity;		/* units of 1% */
			    uint32_t wspeed;	/* average wind speed */
			    uint32_t wgust;		/* wind gust */
#define DAC8FID21_WSPEED_NOT_AVAILABLE		127
			    uint32_t wdir;		/* wind direction */
#define DAC8FID21_WDIR_NOT_AVAILABLE		360
			    uint32_t pressure;	/* air pressure, hpa */
#define DAC8FID21_NONWMO_PRESSURE_NOT_AVAILABLE	403
#define DAC8FID21_NONWMO_PRESSURE_HIGH		402	/* > 1200hPa */
#define DAC8FID21_NONWMO_PRESSURE_OFFSET		400	/* N/A */
			    uint32_t pressuretend;	/* tendency */
		    	    int32_t airtemp;		/* temp, units 0.1C */
#define DAC8FID21_AIRTEMP_NOT_AVAILABLE		-1024
#define DAC8FID21_AIRTEMP_SCALE			10.0
			    uint32_t watertemp;	/* units 0.1degC */
#define DAC8FID21_WATERTEMP_NOT_AVAILABLE	501
#define DAC8FID21_WATERTEMP_SCALE		10.0
			    uint32_t waveperiod;	/* in seconds */
#define DAC8FID21_WAVEPERIOD_NOT_AVAILABLE	63
			    uint32_t wavedir;	/* direction in deg */
#define DAC8FID21_WAVEDIR_NOT_AVAILABLE		360
			    uint32_t swellheight;	/* in decimeters */
			    uint32_t swellperiod;	/* in seconds */
			    uint32_t swelldir;	/* direction in deg */
			} nonwmo_obs;
			struct {
			    int32_t lon;		/* Longitude */
			    int32_t lat;		/* Latitude */
			    uint32_t month;		/* UTC month */
			    uint32_t day;		/* Report day */
			    uint32_t hour;		/* Report hour */
			    uint32_t minute;	/* Report minute */
			    uint32_t course;	/* course over ground */
			    uint32_t speed;		/* speed, m/s */
#define DAC8FID21_SOG_NOT_AVAILABLE		31
#define DAC8FID21_SOG_HIGH_SPEED		30
#define DAC8FID21_SOG_SCALE			2.0
			    uint32_t heading;	/* true heading */
#define DAC8FID21_HDG_NOT_AVAILABLE		127
#define DAC8FID21_HDG_SCALE			5.0
			    uint32_t pressure;	/* units of hPa * 0.1 */
#define DAC8FID21_WMO_PRESSURE_SCALE		10
#define DAC8FID21_WMO_PRESSURE_OFFSET		90.0
			    uint32_t pdelta;	/* units of hPa * 0.1 */
#define DAC8FID21_PDELTA_SCALE			10
#define DAC8FID21_PDELTA_OFFSET			50.0
			    uint32_t ptend;		/* enumerated */
			    uint32_t twinddir;	/* in 5 degree steps */
#define DAC8FID21_TWINDDIR_NOT_AVAILABLE	127
			    uint32_t twindspeed;	/* meters per second */
#define DAC8FID21_TWINDSPEED_SCALE		2
#define DAC8FID21_RWINDSPEED_NOT_AVAILABLE	255
			    uint32_t rwinddir;	/* in 5 degree steps */
#define DAC8FID21_RWINDDIR_NOT_AVAILABLE	127
			    uint32_t rwindspeed;	/* meters per second */
#define DAC8FID21_RWINDSPEED_SCALE		2
#define DAC8FID21_RWINDSPEED_NOT_AVAILABLE	255
			    uint32_t mgustspeed;	/* meters per second */
#define DAC8FID21_MGUSTSPEED_SCALE		2
#define DAC8FID21_MGUSTSPEED_NOT_AVAILABLE	255
			    uint32_t mgustdir;	/* in 5 degree steps */
#define DAC8FID21_MGUSTDIR_NOT_AVAILABLE	127
			    uint32_t airtemp;	/* degress K */
#define DAC8FID21_AIRTEMP_OFFSET		223
			    uint32_t humidity;		/* units of 1% */
#define DAC8FID21_HUMIDITY_NOT_VAILABLE		127
			    /* some trailing fields are missing */
			} wmo_obs;
		    };
	        } dac1fid21;
		/*** WORK IN PROGRESS ENDS HERE ***/
		/* IMO289 - Dangerous Cargo Indication */
		struct {
		    uint32_t unit;	/* Unit of Quantity */
		    uint32_t amount;	/* Amount of Cargo */
		    int32_t ncargos;
		    struct cargo_t {
			uint32_t code;	/* Cargo code */
			uint32_t subtype;	/* Cargo subtype */
		    } cargos[28];
		} dac1fid25;
		/* IMO289 - Route info (addressed) */
		/*struct route_info dac1fid28;*/
		/* IMO289 - Text message (addressed) */
		struct {
		    uint32_t linkage;
#define AIS_DAC1FID30_TEXT_MAX	154	/* 920 bits of six-bit, plus NUL */
		    int8_t text[AIS_DAC1FID30_TEXT_MAX];
		} dac1fid30;
		/* IMO289 & IMO236 - Tidal Window */
		struct {
		    uint32_t type;	/* Message Type */
		    uint32_t repeat;	/* Repeat Indicator */
		    uint32_t mmsi;	/* Source MMSI */
		    uint32_t seqno;	/* Sequence Number */
		    uint32_t dest_mmsi;	/* Destination MMSI */
		    int32_t retransmit;	/* Retransmit flag */
		    uint32_t dac;	/* DAC */
		    uint32_t fid;	/* FID */
		    uint32_t month;	/* Month */
		    uint32_t day;	/* Day */
		    int32_t ntidals;
		    struct tidal_t {
			int32_t lon;	/* Longitude */
			int32_t lat;	/* Latitude */
			uint32_t from_hour;	/* From UTC Hour */
			uint32_t from_min;	/* From UTC Minute */
			uint32_t to_hour;	/* To UTC Hour */
			uint32_t to_min;	/* To UTC Minute */
#define DAC1FID32_CDIR_NOT_AVAILABLE		360
			uint32_t cdir;	/* Current Dir. Predicted */
#define DAC1FID32_CSPEED_NOT_AVAILABLE		127
			uint32_t cspeed;	/* Current Speed Predicted */
		    } tidals[3];
		} dac1fid32;
	    }        dac1;
	} type6;
	/* Type 7 - Binary Acknowledge */
	struct {
	    uint32_t mmsi1;
	    uint32_t mmsi2;
	    uint32_t mmsi3;
	    uint32_t mmsi4;
	    /* spares ignored, they're only padding here */
	} type7;
	/* Type 8 - Broadcast Binary Message */
	struct {
	    uint32_t dac;       	/* Designated Area Code */
	    uint32_t fid;       	/* Functional ID */
#define AIS_TYPE8_BINARY_MAX	952	/* 952 bits */
	    size_t bitcount;		/* bit count of the data */
	    union {
		int8_t bitdata[(AIS_TYPE8_BINARY_MAX + 7) / 8];
		/* IMO236  - Meteorological-Hydrological data
		 * Trial message, not to be used after January 2013
		 * Replaced by IMO289 (DAC 1, FID 31)
		 */
		struct {
#define DAC1FID11_LATLON_SCALE			1000
		    int32_t lon;			/* longitude in minutes * .001 */
#define DAC1FID11_LON_NOT_AVAILABLE		0xFFFFFF
		    int32_t lat;			/* latitude in minutes * .001 */
#define DAC1FID11_LAT_NOT_AVAILABLE		0x7FFFFF
		    uint32_t day;		/* UTC day */
		    uint32_t hour;		/* UTC hour */
		    uint32_t minute;	/* UTC minute */
		    uint32_t wspeed;	/* average wind speed */
		    uint32_t wgust;		/* wind gust */
#define DAC1FID11_WSPEED_NOT_AVAILABLE		127
		    uint32_t wdir;		/* wind direction */
		    uint32_t wgustdir;	/* wind gust direction */
#define DAC1FID11_WDIR_NOT_AVAILABLE		511
		    uint32_t airtemp;	/* temperature, units 0.1C */
#define DAC1FID11_AIRTEMP_NOT_AVAILABLE		2047
#define DAC1FID11_AIRTEMP_OFFSET		600
#define DAC1FID11_AIRTEMP_DIV			10.0
		    uint32_t humidity;	/* relative humidity, % */
#define DAC1FID11_HUMIDITY_NOT_AVAILABLE	127
		    uint32_t dewpoint;	/* dew point, units 0.1C */
#define DAC1FID11_DEWPOINT_NOT_AVAILABLE	1023
#define DAC1FID11_DEWPOINT_OFFSET		200
#define DAC1FID11_DEWPOINT_DIV		10.0
		    uint32_t pressure;	/* air pressure, hpa */
#define DAC1FID11_PRESSURE_NOT_AVAILABLE	511
#define DAC1FID11_PRESSURE_OFFSET		-800
		    uint32_t pressuretend;	/* tendency */
#define DAC1FID11_PRESSURETREND_NOT_AVAILABLE	3
		    uint32_t visibility;	/* units 0.1 nautical miles */
#define DAC1FID11_VISIBILITY_NOT_AVAILABLE	255
#define DAC1FID11_VISIBILITY_DIV		10.0
		    int32_t waterlevel;		/* decimeters */
#define DAC1FID11_WATERLEVEL_NOT_AVAILABLE	511
#define DAC1FID11_WATERLEVEL_OFFSET		100
#define DAC1FID11_WATERLEVEL_DIV		10.0
		    uint32_t leveltrend;	/* water level trend code */
#define DAC1FID11_WATERLEVELTREND_NOT_AVAILABLE	3
		    uint32_t cspeed;	/* surface current speed in deciknots */
#define DAC1FID11_CSPEED_NOT_AVAILABLE		255
#define DAC1FID11_CSPEED_DIV			10.0
		    uint32_t cdir;		/* surface current dir., degrees */
#define DAC1FID11_CDIR_NOT_AVAILABLE		511
		    uint32_t cspeed2;	/* current speed in deciknots */
		    uint32_t cdir2;		/* current dir., degrees */
		    uint32_t cdepth2;	/* measurement depth, m */
#define DAC1FID11_CDEPTH_NOT_AVAILABLE		31
		    uint32_t cspeed3;	/* current speed in deciknots */
		    uint32_t cdir3;		/* current dir., degrees */
		    uint32_t cdepth3;	/* measurement depth, m */
		    uint32_t waveheight;	/* in decimeters */
#define DAC1FID11_WAVEHEIGHT_NOT_AVAILABLE	255
#define DAC1FID11_WAVEHEIGHT_DIV		10.0
		    uint32_t waveperiod;	/* in seconds */
#define DAC1FID11_WAVEPERIOD_NOT_AVAILABLE	63
		    uint32_t wavedir;	/* direction in degrees */
#define DAC1FID11_WAVEDIR_NOT_AVAILABLE		511
		    uint32_t swellheight;	/* in decimeters */
		    uint32_t swellperiod;	/* in seconds */
		    uint32_t swelldir;	/* direction in degrees */
		    uint32_t seastate;	/* Beaufort scale, 0-12 */
#define DAC1FID11_SEASTATE_NOT_AVAILABLE	15
		    uint32_t watertemp;	/* units 0.1deg Celsius */
#define DAC1FID11_WATERTEMP_NOT_AVAILABLE	1023
#define DAC1FID11_WATERTEMP_OFFSET		100
#define DAC1FID11_WATERTEMP_DIV		10.0
		    uint32_t preciptype;	/* 0-7, enumerated */
#define DAC1FID11_PRECIPTYPE_NOT_AVAILABLE	7
		    uint32_t salinity;	/* units of 0.1ppt */
#define DAC1FID11_SALINITY_NOT_AVAILABLE	511
#define DAC1FID11_SALINITY_DIV		10.0
		    uint32_t ice;		/* is there sea ice? */
#define DAC1FID11_ICE_NOT_AVAILABLE		3
		} dac1fid11;
		/* IMO236 - Fairway Closed */
		struct {
		    int8_t reason[20+1];		/* Reason For Closing */
		    int8_t closefrom[20+1];	/* Location Of Closing From */
		    int8_t closeto[20+1];		/* Location of Closing To */
		    uint32_t radius;	/* Radius extension */
#define AIS_DAC1FID13_RADIUS_NOT_AVAILABLE 10001
		    uint32_t extunit;	/* Unit of extension */
#define AIS_DAC1FID13_EXTUNIT_NOT_AVAILABLE 0
		    uint32_t fday;		/* From day (UTC) */
		    uint32_t fmonth;	/* From month (UTC) */
		    uint32_t fhour;		/* From hour (UTC) */
		    uint32_t fminute;	/* From minute (UTC) */
		    uint32_t tday;		/* To day (UTC) */
		    uint32_t tmonth;	/* To month (UTC) */
		    uint32_t thour;		/* To hour (UTC) */
		    uint32_t tminute;	/* To minute (UTC) */
		} dac1fid13;
	        /* IMO236 - Extended ship and voyage data */
		struct {
		    uint32_t airdraught;	/* Air Draught */
		} dac1fid15;
		/* IMO289 - VTS-generated/Synthetic Targets */
		struct {
		    int32_t ntargets;
		    struct target_t {
#define DAC1FID17_IDTYPE_MMSI		0
#define DAC1FID17_IDTYPE_IMO		1
#define DAC1FID17_IDTYPE_CALLSIGN	2
#define DAC1FID17_IDTYPE_OTHER		3
			uint32_t idtype;	/* Identifier type */
			union target_id {	/* Target identifier */
			    uint32_t mmsi;
			    uint32_t imo;
#define DAC1FID17_ID_LENGTH		7
			    int8_t callsign[DAC1FID17_ID_LENGTH+1];
			    int8_t other[DAC1FID17_ID_LENGTH+1];
			} id;
			int32_t lat;		/* Latitude */
			int32_t lon;		/* Longitude */
#define DAC1FID17_COURSE_NOT_AVAILABLE		360
			uint32_t course;	/* Course Over Ground */
			uint32_t second;	/* Time Stamp */
#define DAC1FID17_SPEED_NOT_AVAILABLE		255
			uint32_t speed;	/* Speed Over Ground */
		    } targets[4];
		} dac1fid17;
		/* IMO 289 - Marine Traffic Signal */
		struct {
		    uint32_t linkage;	/* Message Linkage ID */
		    int8_t station[20+1];		/* Name of Signal Station */
		    int32_t lon;		/* Longitude */
		    int32_t lat;		/* Latitude */
		    uint32_t status;	/* Status of Signal */
		    uint32_t signal;	/* Signal In Service */
		    uint32_t hour;		/* UTC hour */
		    uint32_t minute;	/* UTC minute */
		    uint32_t nextsignal;	/* Expected Next Signal */
		} dac1fid19;
		/* IMO289 - Route info (broadcast) */
		/*struct route_info dac1fid27;*/
		/* IMO289 - Text message (broadcast) */
		struct {
		    uint32_t linkage;
#define AIS_DAC1FID29_TEXT_MAX	162	/* 920 bits of six-bit, plus NUL */
		    int8_t text[AIS_DAC1FID29_TEXT_MAX];
		} dac1fid29;
		/* IMO289 - Meteorological-Hydrological data */
		struct {
		    bool accuracy;	/* position accuracy, <10m if true */
#define DAC1FID31_LATLON_SCALE	1000
		    int32_t lon;		/* longitude in minutes * .001 */
#define DAC1FID31_LON_NOT_AVAILABLE	(181*60*DAC1FID31_LATLON_SCALE)
		    int32_t lat;		/* longitude in minutes * .001 */
#define DAC1FID31_LAT_NOT_AVAILABLE	(91*60*DAC1FID31_LATLON_SCALE)
		    uint32_t day;		/* UTC day */
		    uint32_t hour;		/* UTC hour */
		    uint32_t minute;	/* UTC minute */
		    uint32_t wspeed;	/* average wind speed */
		    uint32_t wgust;		/* wind gust */
#define DAC1FID31_WIND_HIGH			126
#define DAC1FID31_WIND_NOT_AVAILABLE		127
		    uint32_t wdir;		/* wind direction */
		    uint32_t wgustdir;	/* wind gust direction */
#define DAC1FID31_DIR_NOT_AVAILABLE		360
		    int32_t airtemp;		/* temperature, units 0.1C */
#define DAC1FID31_AIRTEMP_NOT_AVAILABLE		-1024
#define DAC1FID31_AIRTEMP_DIV			10.0
		    uint32_t humidity;	/* relative humidity, % */
#define DAC1FID31_HUMIDITY_NOT_AVAILABLE	101
		    int32_t dewpoint;		/* dew point, units 0.1C */
#define DAC1FID31_DEWPOINT_NOT_AVAILABLE	501
#define DAC1FID31_DEWPOINT_DIV		10.0
		    uint32_t pressure;	/* air pressure, hpa */
#define DAC1FID31_PRESSURE_NOT_AVAILABLE	511
#define DAC1FID31_PRESSURE_HIGH			402
#define DAC1FID31_PRESSURE_OFFSET		-799
		    uint32_t pressuretend;	/* tendency */
#define DAC1FID31_PRESSURETEND_NOT_AVAILABLE	3
		    bool visgreater;            /* visibility greater than */
		    uint32_t visibility;	/* units 0.1 nautical miles */
#define DAC1FID31_VISIBILITY_NOT_AVAILABLE	127
#define DAC1FID31_VISIBILITY_DIV		10.0
		    int32_t waterlevel;		/* cm */
#define DAC1FID31_WATERLEVEL_NOT_AVAILABLE	4001
#define DAC1FID31_WATERLEVEL_OFFSET		1000
#define DAC1FID31_WATERLEVEL_DIV		100.0
		    uint32_t leveltrend;	/* water level trend code */
#define DAC1FID31_WATERLEVELTREND_NOT_AVAILABLE	3
		    uint32_t cspeed;	/* current speed in deciknots */
#define DAC1FID31_CSPEED_NOT_AVAILABLE		255
#define DAC1FID31_CSPEED_DIV			10.0
		    uint32_t cdir;		/* current dir., degrees */
		    uint32_t cspeed2;	/* current speed in deciknots */
		    uint32_t cdir2;		/* current dir., degrees */
		    uint32_t cdepth2;	/* measurement depth, 0.1m */
#define DAC1FID31_CDEPTH_NOT_AVAILABLE		301
#define DAC1FID31_CDEPTH_SCALE			10.0
		    uint32_t cspeed3;	/* current speed in deciknots */
		    uint32_t cdir3;		/* current dir., degrees */
		    uint32_t cdepth3;	/* measurement depth, 0.1m */
		    uint32_t waveheight;	/* in decimeters */
#define DAC1FID31_HEIGHT_NOT_AVAILABLE		31
#define DAC1FID31_HEIGHT_DIV			10.0
		    uint32_t waveperiod;	/* in seconds */
#define DAC1FID31_PERIOD_NOT_AVAILABLE		63
		    uint32_t wavedir;	/* direction in degrees */
		    uint32_t swellheight;	/* in decimeters */
		    uint32_t swellperiod;	/* in seconds */
		    uint32_t swelldir;	/* direction in degrees */
		    uint32_t seastate;	/* Beaufort scale, 0-12 */
#define DAC1FID31_SEASTATE_NOT_AVAILABLE	15
		    int32_t watertemp;		/* units 0.1deg Celsius */
#define DAC1FID31_WATERTEMP_NOT_AVAILABLE	601
#define DAC1FID31_WATERTEMP_DIV		10.0
		    uint32_t preciptype;	/* 0-7, enumerated */
#define DAC1FID31_PRECIPTYPE_NOT_AVAILABLE	7
		    uint32_t salinity;	/* units of 0.1 permil (ca. PSU) */
#define DAC1FID31_SALINITY_NOT_AVAILABLE	510
#define DAC1FID31_SALINITY_DIV		10.0
		    uint32_t ice;		/* is there sea ice? */
#define DAC1FID31_ICE_NOT_AVAILABLE		3
		} dac1fid31;
	    }dac1;
	} type8;
	/* Type 9 - Standard SAR Aircraft Position Report */
	struct {
	    uint32_t alt;		/* altitude in meters */
#define AIS_ALT_NOT_AVAILABLE	4095
#define AIS_ALT_HIGH    	4094	/* 4094 meters or higher */
	    uint32_t speed;		/* speed over ground in deciknots */
#define AIS_SAR_SPEED_NOT_AVAILABLE	1023
#define AIS_SAR_FAST_MOVER  	1022
	    bool accuracy;		/* position accuracy */
	    int32_t lon;			/* longitude */
	    int32_t lat;			/* latitude */
	    uint32_t course;	/* course over ground */
	    uint32_t second;	/* seconds of UTC timestamp */
	    uint32_t regional;	/* regional reserved */
	    uint32_t dte;		/* data terminal enable */
	    //uint32_t spare;	spare bits */
	    bool assigned;		/* assigned-mode flag */
	    bool raim;			/* RAIM flag */
	    uint32_t radio;		/* radio status bits */
	} type9;
	/* Type 10 - UTC/Date Inquiry */
	struct {
	    //uint32_t spare;
	    uint32_t dest_mmsi;	/* destination MMSI */
	    //uint32_t spare2;
	} type10;
	/* Type 12 - Safety-Related Message */
	struct {
	    uint32_t seqno;		/* sequence number */
	    uint32_t dest_mmsi;	/* destination MMSI */
	    bool retransmit;		/* retransmit flag */
	    //uint32_t spare;	spare bit(s) */
#define AIS_TYPE12_TEXT_MAX	157	/* 936 bits of six-bit, plus NUL */
	    int8_t text[AIS_TYPE12_TEXT_MAX];
	} type12;
	/* Type 14 - Safety-Related Broadcast Message */
	struct {
	    //uint32_t spare;	spare bit(s) */
#define AIS_TYPE14_TEXT_MAX	161	/* 952 bits of six-bit, plus NUL */
	    int8_t text[AIS_TYPE14_TEXT_MAX];
	} type14;
	/* Type 15 - Interrogation */
	struct {
	    //uint32_t spare;	spare bit(s) */
	    uint32_t mmsi1;
	    uint32_t type1_1;
	    uint32_t offset1_1;
	    //uint32_t spare2;	spare bit(s) */
	    uint32_t type1_2;
	    uint32_t offset1_2;
	    //uint32_t spare3;	spare bit(s) */
	    uint32_t mmsi2;
	    uint32_t type2_1;
	    uint32_t offset2_1;
	    //uint32_t spare4;	spare bit(s) */
	} type15;
	/* Type 16 - Assigned Mode Command */
	struct {
	    //uint32_t spare;	spare bit(s) */
	    uint32_t mmsi1;
	    uint32_t offset1;
	    uint32_t increment1;
	    uint32_t mmsi2;
	    uint32_t offset2;
	    uint32_t increment2;
	} type16;
	/* Type 17 - GNSS Broadcast Binary Message */
	struct {
	    //uint32_t spare;	spare bit(s) */
#define AIS_GNSS_LATLON_DIV	600.0
	    int32_t lon;			/* longitude */
	    int32_t lat;			/* latitude */
	    //uint32_t spare2;	spare bit(s) */
#define AIS_TYPE17_BINARY_MAX	736	/* 920 bits */
	    size_t bitcount;		/* bit count of the data */
	    int8_t bitdata[(AIS_TYPE17_BINARY_MAX + 7) / 8];
	} type17;
	/* Type 18 - Standard Class B CS Position Report */
	struct {
	    uint32_t reserved;	/* altitude in meters */
	    uint32_t speed;		/* speed over ground in deciknots */
	    bool accuracy;		/* position accuracy */
	    int32_t lon;			/* longitude */
#define AIS_GNS_LON_NOT_AVAILABLE	0x1a838
	    int32_t lat;			/* latitude */
#define AIS_GNS_LAT_NOT_AVAILABLE	0xd548
	    uint32_t course;	/* course over ground */
	    uint32_t heading;	/* true heading */
	    uint32_t second;	/* seconds of UTC timestamp */
	    uint32_t regional;	/* regional reserved */
	    bool cs;     		/* carrier sense unit flag */
	    bool display;		/* unit has attached display? */
	    bool dsc;   		/* unit attached to radio with DSC? */
	    bool band;   		/* unit can switch frequency bands? */
	    bool msg22;	        	/* can accept Message 22 management? */
	    bool assigned;		/* assigned-mode flag */
	    bool raim;			/* RAIM flag */
	    uint32_t radio;		/* radio status bits */
	} type18;
	/* Type 19 - Extended Class B CS Position Report */
	struct {
	    uint32_t reserved;	/* altitude in meters */
	    uint32_t speed;		/* speed over ground in deciknots */
	    bool accuracy;		/* position accuracy */
	    int32_t lon;			/* longitude */
	    int32_t lat;			/* latitude */
	    uint32_t course;	/* course over ground */
	    uint32_t heading;	/* true heading */
	    uint32_t second;	/* seconds of UTC timestamp */
	    uint32_t regional;	/* regional reserved */
	    int8_t shipname[AIS_SHIPNAME_MAXLEN+1];		/* ship name */
	    uint32_t shiptype;	/* ship type code */
	    uint32_t to_bow;	/* dimension to bow */
	    uint32_t to_stern;	/* dimension to stern */
	    uint32_t to_port;	/* dimension to port */
	    uint32_t to_starboard;	/* dimension to starboard */
	    uint32_t epfd;		/* type of position fix deviuce */
	    bool raim;			/* RAIM flag */
	    uint32_t dte;    	/* date terminal enable */
	    bool assigned;		/* assigned-mode flag */
	    //uint32_t spare;	spare bits */
	} type19;
	/* Type 20 - Data Link Management Message */
	struct {
	    //uint32_t spare;	spare bit(s) */
	    uint32_t offset1;	/* TDMA slot offset */
	    uint32_t number1;	/* number of xlots to allocate */
	    uint32_t timeout1;	/* allocation timeout */
	    uint32_t increment1;	/* repeat increment */
	    uint32_t offset2;	/* TDMA slot offset */
	    uint32_t number2;	/* number of xlots to allocate */
	    uint32_t timeout2;	/* allocation timeout */
	    uint32_t increment2;	/* repeat increment */
	    uint32_t offset3;	/* TDMA slot offset */
	    uint32_t number3;	/* number of xlots to allocate */
	    uint32_t timeout3;	/* allocation timeout */
	    uint32_t increment3;	/* repeat increment */
	    uint32_t offset4;	/* TDMA slot offset */
	    uint32_t number4;	/* number of xlots to allocate */
	    uint32_t timeout4;	/* allocation timeout */
	    uint32_t increment4;	/* repeat increment */
	} type20;
	/* Type 21 - Aids to Navigation Report */
	struct {
	    uint32_t aid_type;	/* aid type */
	    int8_t name[35];		/* name of aid to navigation */
	    bool accuracy;		/* position accuracy */
	    int32_t lon;			/* longitude */
	    int32_t lat;			/* latitude */
	    uint32_t to_bow;	/* dimension to bow */
	    uint32_t to_stern;	/* dimension to stern */
	    uint32_t to_port;	/* dimension to port */
	    uint32_t to_starboard;	/* dimension to starboard */
	    uint32_t epfd;		/* type of EPFD */
	    uint32_t second;	/* second of UTC timestamp */
	    bool off_position;		/* off-position indicator */
	    uint32_t regional;	/* regional reserved field */
	    bool raim;			/* RAIM flag */
	    bool virtual_aid;		/* is virtual station? */
	    bool assigned;		/* assigned-mode flag */
	    //uint32_t spare;	unused */
	} type21;
	/* Type 22 - Channel Management */
	struct {
	    //uint32_t spare;	spare bit(s) */
	    uint32_t channel_a;	/* Channel A number */
	    uint32_t channel_b;	/* Channel B number */
	    uint32_t txrx;		/* transmit/receive mode */
	    bool power;			/* high-power flag */
#define AIS_CHANNEL_LATLON_DIV	600.0
	    union {
		struct {
		    int32_t ne_lon;		/* NE corner longitude */
		    int32_t ne_lat;		/* NE corner latitude */
		    int32_t sw_lon;		/* SW corner longitude */
		    int32_t sw_lat;		/* SW corner latitude */
		} area;
		struct {
		    uint32_t dest1;	/* addressed station MMSI 1 */
		    uint32_t dest2;	/* addressed station MMSI 2 */
		} mmsi;
	    }area_mmsi;
	    bool addressed;		/* addressed vs. broadast flag */
	    bool band_a;		/* fix 1.5kHz band for channel A */
	    bool band_b;		/* fix 1.5kHz band for channel B */
	    uint32_t zonesize;	/* size of transitional zone */
	} type22;
	/* Type 23 - Group Assignment Command */
	struct {
	    int32_t ne_lon;			/* NE corner longitude */
	    int32_t ne_lat;			/* NE corner latitude */
	    int32_t sw_lon;			/* SW corner longitude */
	    int32_t sw_lat;			/* SW corner latitude */
	    //uint32_t spare;	spare bit(s) */
	    uint32_t stationtype;	/* station type code */
	    uint32_t shiptype;	/* ship type code */
	    //uint32_t spare2;	spare bit(s) */
	    uint32_t txrx;		/* transmit-enable code */
	    uint32_t interval;	/* report interval */
	    uint32_t quiet;		/* quiet time */
	    //uint32_t spare3;	spare bit(s) */
	} type23;
	/* Type 24 - Class B CS Static Data Report */
	struct {
	    int8_t shipname[AIS_SHIPNAME_MAXLEN+1];	/* vessel name */
	    uint32_t shiptype;	/* ship type code */
	    int8_t vendorid[8];		/* vendor ID */
	    int8_t callsign[8];		/* callsign */
	    union {
		uint32_t mothership_mmsi;	/* MMSI of main vessel */
		struct {
		    uint32_t to_bow;	/* dimension to bow */
		    uint32_t to_stern;	/* dimension to stern */
		    uint32_t to_port;	/* dimension to port */
		    uint32_t to_starboard;	/* dimension to starboard */
		} dim;
	    };
	} type24;
	/* Type 25 - Addressed Binary Message */
	struct {
	    bool addressed;		/* addressed-vs.broadcast flag */
	    bool structured;		/* structured-binary flag */
	    uint32_t dest_mmsi;	/* destination MMSI */
	    uint32_t app_id;        /* Application ID */
#define AIS_TYPE25_BINARY_MAX	128	/* Up to 128 bits */
	    size_t bitcount;		/* bit count of the data */
	    int8_t bitdata[(AIS_TYPE25_BINARY_MAX + 7) / 8];
	} type25;
	/* Type 26 - Addressed Binary Message */
	struct {
	    bool addressed;		/* addressed-vs.broadcast flag */
	    bool structured;		/* structured-binary flag */
	    uint32_t dest_mmsi;	/* destination MMSI */
	    uint32_t app_id;        /* Application ID */
#define AIS_TYPE26_BINARY_MAX	1004	/* Up to 128 bits */
	    size_t bitcount;		/* bit count of the data */
	    int8_t bitdata[(AIS_TYPE26_BINARY_MAX + 7) / 8];
	    uint32_t radio;		/* radio status bits */
	} type26;
	/* Type 27 - Long Range AIS Broadcast message */
	struct {
	    bool accuracy;		/* position accuracy */
	    bool raim;			/* RAIM flag */
	    uint32_t status;	/* navigation status */
#define AIS_LONGRANGE_LATLON_DIV	600.0
	    int32_t lon;			/* longitude */
#define AIS_LONGRANGE_LON_NOT_AVAILABLE	0x1a838
	    int32_t lat;			/* latitude */
#define AIS_LONGRANGE_LAT_NOT_AVAILABLE	0xd548
	    uint32_t speed;		/* speed over ground in deciknots */
#define AIS_LONGRANGE_SPEED_NOT_AVAILABLE 63
	    uint32_t course;	/* course over ground */
#define AIS_LONGRANGE_COURSE_NOT_AVAILABLE 511
	    bool gnss;			/* are we reporting GNSS position? */
	} type27;
    }types;
};

struct gps_data_t {
#define ONLINE_SET	(1llu<<1)
#define TIME_SET	(1llu<<2)
#define TIMERR_SET	(1llu<<3)
#define LATLON_SET	(1llu<<4)
#define ALTITUDE_SET	(1llu<<5)
#define SPEED_SET	(1llu<<6)
#define TRACK_SET	(1llu<<7)
#define CLIMB_SET	(1llu<<8)
#define STATUS_SET	(1llu<<9)
#define MODE_SET	(1llu<<10)
#define DOP_SET  	(1llu<<11)
#define HERR_SET	(1llu<<12)
#define VERR_SET	(1llu<<13)
#define ATTITUDE_SET	(1llu<<14)
#define SATELLITE_SET	(1llu<<15)
#define SPEEDERR_SET	(1llu<<16)
#define TRACKERR_SET	(1llu<<17)
#define CLIMBERR_SET	(1llu<<18)
#define DEVICE_SET	(1llu<<19)
#define DEVICELIST_SET	(1llu<<20)
#define DEVICEID_SET	(1llu<<21)
#define RTCM2_SET	(1llu<<22)
#define RTCM3_SET	(1llu<<23)
#define AIS_SET 	(1llu<<24)
#define PACKET_SET	(1llu<<25)
#define SUBFRAME_SET	(1llu<<26)
#define GST_SET 	(1llu<<27)
#define VERSION_SET	(1llu<<28)
#define POLICY_SET	(1llu<<29)
#define LOGMESSAGE_SET	(1llu<<30)
#define ERROR_SET	(1llu<<31)
#define SET_HIGH_BIT	31
	struct ais_t ais;
};


/* ghye */
#define MAX_NMEA_FILED 20

struct nmea_time_t{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
};

enum nmea_type_t{
	TYPE_ERROR,
	TYPE_GPGGA,
	TYPE_GPRMC,
	TYPE_GPVTG,
};

enum nmea_status_t{
	STATUS_ERROR,
	STATUS_NO_FIX,
	STATUS_FIX,
};

struct nmea_t{
	enum nmea_type_t type;
	int8_t *filed[MAX_NMEA_FILED];
	int8_t filedlen;
	int8_t buf[256];
	union{
		struct{
			enum nmea_status_t status;
			struct nmea_time_t time;
			double lat;
			double lon;
			double speed;	/*Speed over ground, knots*/
			double track;	/*Track made good, degrees true*/
			double magnetic;	/*Magnetic Variation, degrees*/
		}gprmc;
		struct{
			struct nmea_time_t time;
			double lat;
			double lon;
			enum{
				FIX_UNDEF = -1,
				FIX_NOT_AVAIL,
				FIX,
				DIFF_FIX,
			}gps_quality;
			uint8_t nsv;	/* Number of satellites in view, 00 - 12 */
			double hdp;	/* Horizontal Dilution of precision */
			double antenna;	/* Antenna Altitude above/below mena-sea-level (geoid) */
			double geoidal;
			double age;
			uint32_t stationID;
		}gpgga;
		struct{
			double t_track; /* Track Degrees */
			double m_track;
			double knots_speed;
			double killmeters_speed;
		}gpvtg;
	}nmea_u;
};

struct gps_device_t{
    struct gps_packet_t packet;
    struct aivdm_context_t aivdm[AIVDM_CHANNELS];
    struct gps_data_t gpsdata;
	struct nmea_t nmea;	/*ghye*/
};

int32_t nmea_parse(int8_t *buf, struct gps_device_t *session);
uint64_t /*gps_mask_t*/ aivdm_analyze(struct gps_device_t *session);

#endif

