#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <assert.h>
#include "proto.h"
//#include "nmea0813.h"

void gpsd_report(int32_t errlevel/* UNUSED*/, const int8_t *fmt, ... )
/* our version of the logger */
{
#if 0
    int8_t *err_str;
    if (errlevel <= debuglevel) {
	va_list ap;
	va_start(ap, fmt);
	switch ( errlevel ) {
	case LOG_ERROR:
		err_str = "ERROR: ";
		break;
	case LOG_SHOUT:
		err_str = "SHOUT: ";
		break;
	case LOG_WARN:
		err_str = "WARN: ";
		break;
	case LOG_INF:
		err_str = "INFO: ";
		break;
	case LOG_DATA:
		err_str = "DATA: ";
		break;
	case LOG_PROG:
		err_str = "PROG: ";
		break;
	case LOG_IO:
		err_str = "IO: ";
		break;
	case LOG_SPIN:
		err_str = "SPIN: ";
		break;
	case LOG_RAW:
		err_str = "RAW: ";
		break;
	default:
		err_str = "UNK: ";
	}

	(void)fputs("gpsctl:", stderr);
	(void)fputs(err_str, stderr);
	(void)vfprintf(stderr, fmt, ap);
	va_end(ap);
    }
#endif
}

uint64_t ubits(int8_t buf[], uint32_t start, uint32_t width, bool le)
/* extract a (zero-origin) bitfield from the buffer as an unsigned big-endian uint64_t */
{
    uint64_t fld = 0;
    uint32_t i;
    uint32_t end;

    /*@i1@*/ assert(width <= sizeof(uint64_t) * CHAR_BIT);
    for (i = start / CHAR_BIT;
	 i < (start + width + CHAR_BIT - 1) / CHAR_BIT; i++) {
	/*@i1@*/fld <<= CHAR_BIT;
	fld |= (uint8_t)buf[i];
    }

    end = (start + width) % CHAR_BIT;
    if (end != 0) {
	/*@i1@*/fld >>= (CHAR_BIT - end);
    }

    /*@ -shiftimplementation @*/
    fld &= ~(-1LL << width);
    /*@ +shiftimplementation @*/

    /* was extraction as a little-endian requested? */
    if (le)
    {
	uint64_t reversed = 0;

	for (i = width; i; --i)
	{
	    reversed <<= 1;
	    if (fld & 1)
		reversed |= 1;
	    fld >>= 1;
	}
	fld = reversed;
    }

    return fld;
}

int64_t sbits(int8_t buf[], uint32_t start, uint32_t width, bool le)
/* extract a bitfield from the buffer as a signed big-endian long */
{
    uint64_t fld = ubits(buf, start, width, le);

    /*@ +relaxtypes */
    if (fld & (1LL << (width - 1))) {
	/*@ -shiftimplementation @*/
	fld |= (-1LL << (width - 1));
	/*@ +shiftimplementation @*/
    }
    return (int64_t)fld;
    /*@ -relaxtypes */
}

/*
 * Parse the data from the device
 */
static void from_sixbit(int8_t *bitvec, uint32_t start, int32_t count, int8_t *to)
{
    /*@ +type @*/
#ifdef S_SPLINT_S
    /* the real string causes a splint internal error */
    const int8_t sixchr[] = "abcd";
#else
    const int8_t sixchr[64] =
	"@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^- !\"#$%&'()*+,-./0123456789:;<=>?";
#endif /* S_SPLINT_S */
    int32_t i;
    int8_t newchar;

    /* six-bit to ASCII */
    for (i = 0; i < count - 1; i++) {
	newchar = sixchr[ubits(bitvec, start + 6 * i, 6U, false)];
	if (newchar == '@')
	    break;
	else
	    to[i] = newchar;
    }
    to[i] = '\0';
    /* trim spaces on right end */
    for (i = count - 2; i >= 0; i--)
	if (to[i] == ' ' || to[i] == '@')
	    to[i] = '\0';
	else
	    break;
    /*@ -type @*/
}

/*@ +charint @*/
bool ais_binary_decode(struct ais_t *ais,
			      const uint8_t *bits, size_t bitlen)
			      /*struct ais_type24_queue_t *type24_queue)*/
/* decode an AIS binary packet */
{
    bool imo;
    uint32_t u;
    /*int32_t i;*/

#define BITS_PER_BYTE	8
#define UBITS(s, l)	ubits((int8_t *)bits, s, l, false)
#define SBITS(s, l)	sbits((int8_t *)bits, s, l, false)
#define UCHARS(s, to)	from_sixbit((int8_t *)bits, s, sizeof(to), to)
    ais->type = UBITS(0, 6);
    ais->repeat = UBITS(6, 2);
    ais->mmsi = UBITS(8, 30);
    gpsd_report(LOG_INF, "AIVDM message type %d, MMSI %09d:\n",
		ais->type, ais->mmsi);
    /*
     * Something about the shape of this switch statement confuses
     * GNU indent so badly that there is no point in trying to be
     * finer-grained than leaving it all alone.
     */
    /* *INDENT-OFF* */
    switch (ais->type) {
    case 1:	/* Position Report */
    case 2:
    case 3:
	if (bitlen != 168) {
	    gpsd_report(LOG_WARN, "AIVDM message type %d size not 168 bits (%zd).\n",
			ais->type,
			bitlen);
	    return false;
	}
	ais->types.type1.status		= UBITS(38, 4);
	ais->types.type1.turn		= SBITS(42, 8);
	ais->types.type1.speed		= UBITS(50, 10);
	ais->types.type1.accuracy	        = UBITS(60, 1)!=0;
	ais->types.type1.lon		= SBITS(61, 28);
	ais->types.type1.lat		= SBITS(89, 27);
	ais->types.type1.course		= UBITS(116, 12);
	ais->types.type1.heading	        = UBITS(128, 9);
	ais->types.type1.second		= UBITS(137, 6);
	ais->types.type1.maneuver	        = UBITS(143, 2);
	//ais->types.type1.spare	        = UBITS(145, 3);
	ais->types.type1.raim		= UBITS(148, 1)!=0;
	ais->types.type1.radio		= UBITS(149, 20);
	break;
    case 4: 	/* Base Station Report */
    case 11:	/* UTC/Date Response */
	if (bitlen != 168) {
	    gpsd_report(LOG_WARN, "AIVDM message type %d size not 168 bits (%zd).\n",
			ais->type,
			bitlen);
	    return false;
	}
	ais->types.type4.year		= UBITS(38, 14);
	ais->types.type4.month		= UBITS(52, 4);
	ais->types.type4.day		= UBITS(56, 5);
	ais->types.type4.hour		= UBITS(61, 5);
	ais->types.type4.minute		= UBITS(66, 6);
	ais->types.type4.second		= UBITS(72, 6);
	ais->types.type4.accuracy		= UBITS(78, 1)!=0;
	ais->types.type4.lon		= SBITS(79, 28);
	ais->types.type4.lat		= SBITS(107, 27);
	ais->types.type4.epfd		= UBITS(134, 4);
	//ais->types.type4.spare		= UBITS(138, 10);
	ais->types.type4.raim		= UBITS(148, 1)!=0;
	ais->types.type4.radio		= UBITS(149, 19);
	break;
    case 5: /* Ship static and voyage related data */
	if (bitlen != 424) {
	    gpsd_report(LOG_WARN, "AIVDM message type 5 size not 424 bits (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type5.ais_version  = UBITS(38, 2);
	ais->types.type5.imo          = UBITS(40, 30);
	UCHARS(70, ais->types.type5.callsign);
	UCHARS(112, ais->types.type5.shipname);
	ais->types.type5.shiptype     = UBITS(232, 8);
	ais->types.type5.to_bow       = UBITS(240, 9);
	ais->types.type5.to_stern     = UBITS(249, 9);
	ais->types.type5.to_port      = UBITS(258, 6);
	ais->types.type5.to_starboard = UBITS(264, 6);
	ais->types.type5.epfd         = UBITS(270, 4);
	ais->types.type5.month        = UBITS(274, 4);
	ais->types.type5.day          = UBITS(278, 5);
	ais->types.type5.hour         = UBITS(283, 5);
	ais->types.type5.minute       = UBITS(288, 6);
	ais->types.type5.draught      = UBITS(294, 8);
	UCHARS(302, ais->types.type5.destination);
	ais->types.type5.dte          = UBITS(422, 1);
	//ais->types.type5.spare        = UBITS(423, 1);
	break;
    case 6: /* Addressed Binary Message */
	if (bitlen < 88 || bitlen > 1008) {
	    gpsd_report(LOG_WARN, "AIVDM message type 6 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type6.seqno          = UBITS(38, 2);
	ais->types.type6.dest_mmsi      = UBITS(40, 30);
	ais->types.type6.retransmit     = UBITS(70, 1)!=0;
	//ais->types.type6.spare        = UBITS(71, 1);
	ais->types.type6.dac            = UBITS(72, 10);
	ais->types.type6.fid            = UBITS(82, 6);
	ais->types.type6.bitcount       = bitlen - 88;
	imo = false;
	/* UK and Republic Of Ireland */
	if (ais->types.type6.dac == 235 || ais->types.type6.dac == 250) {
	    switch (ais->types.type6.fid) {
	    case 10:	/* GLA - AtoN monitoring data */
		ais->types.type6.dac1.dac235fid10.ana_int		= UBITS(88, 10);
		ais->types.type6.dac1.dac235fid10.ana_ext1		= UBITS(98, 10);
		ais->types.type6.dac1.dac235fid10.ana_ext2		= UBITS(108, 10);
		ais->types.type6.dac1.dac235fid10.racon   = UBITS(118, 2);
		ais->types.type6.dac1.dac235fid10.light   = UBITS(120, 2);
		ais->types.type6.dac1.dac235fid10.alarm  = UBITS(122, 1);
		ais->types.type6.dac1.dac235fid10.stat_ext		= UBITS(123, 8);
		ais->types.type6.dac1.dac235fid10.off_pos	= UBITS(131, 1);
		/* skip 4 bits */
		imo = true;
	    }
	    break;
	}
	/* International */
	else if (ais->types.type6.dac == 1)
	    switch (ais->types.type6.fid) {
	    case 12:	/* IMO236 - Dangerous cargo indication */
		UCHARS(88, ais->types.type6.dac1.dac1fid12.lastport);
		ais->types.type6.dac1.dac1fid12.lmonth		= UBITS(118, 4);
		ais->types.type6.dac1.dac1fid12.lday		= UBITS(122, 5);
		ais->types.type6.dac1.dac1fid12.lhour		= UBITS(127, 5);
		ais->types.type6.dac1.dac1fid12.lminute	= UBITS(132, 6);
		UCHARS(138, ais->types.type6.dac1.dac1fid12.nextport);
		ais->types.type6.dac1.dac1fid12.nmonth		= UBITS(168, 4);
		ais->types.type6.dac1.dac1fid12.nday		= UBITS(172, 5);
		ais->types.type6.dac1.dac1fid12.nhour		= UBITS(177, 5);
		ais->types.type6.dac1.dac1fid12.nminute	= UBITS(182, 6);
		UCHARS(188, ais->types.type6.dac1.dac1fid12.dangerous);
		UCHARS(308, ais->types.type6.dac1.dac1fid12.imdcat);
		ais->types.type6.dac1.dac1fid12.unid		= UBITS(332, 13);
		ais->types.type6.dac1.dac1fid12.amount		= UBITS(345, 10);
		ais->types.type6.dac1.dac1fid12.unit		= UBITS(355, 2);
		/* skip 3 bits */
		imo = true;
		break;
	    case 14:	/* IMO236 - Tidal Window */
		ais->types.type6.dac1.dac1fid32.month	= UBITS(88, 4);
		ais->types.type6.dac1.dac1fid32.day	= UBITS(92, 5);
#define ARRAY_BASE 97
#define ELEMENT_SIZE 93
		for (u = 0; ARRAY_BASE + (ELEMENT_SIZE*u) <= bitlen; u++) {
		    int32_t a = ARRAY_BASE + (ELEMENT_SIZE*u);
		    struct tidal_t *tp = &ais->types.type6.dac1.dac1fid32.tidals[u];
		    tp->lat	= SBITS(a + 0, 27);
		    tp->lon	= SBITS(a + 27, 28);
		    tp->from_hour	= UBITS(a + 55, 5);
		    tp->from_min	= UBITS(a + 60, 6);
		    tp->to_hour	= UBITS(a + 66, 5);
		    tp->to_min	= UBITS(a + 71, 6);
		    tp->cdir	= UBITS(a + 77, 9);
		    tp->cspeed	= UBITS(a + 86, 7);
		}
		ais->types.type6.dac1.dac1fid32.ntidals = u;
#undef ARRAY_BASE
#undef ELEMENT_SIZE
		imo = true;
		break;
	    case 15:	/* IMO236 - Extended Ship Static and Voyage Related Data */
		ais->types.type6.dac1.dac1fid15.airdraught	= UBITS(56, 11);
		imo = true;
		break;
	    case 16:	/* IMO236 - Number of persons on board */
		if (ais->types.type6.bitcount == 136)
		    ais->types.type6.dac1.dac1fid16.persons = UBITS(88, 13);/* 289 */
		else
		    ais->types.type6.dac1.dac1fid16.persons = UBITS(55, 13);/* 236 */
		imo = true;
		break;
	    case 18:	/* IMO289 - Clearance time to enter port */
		ais->types.type6.dac1.dac1fid18.linkage	= UBITS(88, 10);
		ais->types.type6.dac1.dac1fid18.month	= UBITS(98, 4);
		ais->types.type6.dac1.dac1fid18.day	= UBITS(102, 5);
		ais->types.type6.dac1.dac1fid18.hour	= UBITS(107, 5);
		ais->types.type6.dac1.dac1fid18.minute	= UBITS(112, 6);
		UCHARS(118, ais->types.type6.dac1.dac1fid18.portname);
		UCHARS(238, ais->types.type6.dac1.dac1fid18.destination);
		ais->types.type6.dac1.dac1fid18.lon	= SBITS(268, 25);
		ais->types.type6.dac1.dac1fid18.lat	= SBITS(293, 24);
		/* skip 43 bits */
		imo = true;
		break;
	    case 20:	/* IMO289 - Berthing data - addressed */
		ais->types.type6.dac1.dac1fid20.linkage	= UBITS(88, 10);
		ais->types.type6.dac1.dac1fid20.berth_length	= UBITS(98, 9);
		ais->types.type6.dac1.dac1fid20.berth_depth	= UBITS(107, 8);
		ais->types.type6.dac1.dac1fid20.position	= UBITS(115, 3);
		ais->types.type6.dac1.dac1fid20.month		= UBITS(118, 4);
		ais->types.type6.dac1.dac1fid20.day		= UBITS(122, 5);
		ais->types.type6.dac1.dac1fid20.hour		= UBITS(127, 5);
		ais->types.type6.dac1.dac1fid20.minute		= UBITS(132, 6);
		ais->types.type6.dac1.dac1fid20.availability	= UBITS(138, 1);
		ais->types.type6.dac1.dac1fid20.agent		= UBITS(139, 2);
		ais->types.type6.dac1.dac1fid20.fuel		= UBITS(141, 2);
		ais->types.type6.dac1.dac1fid20.chandler	= UBITS(143, 2);
		ais->types.type6.dac1.dac1fid20.stevedore	= UBITS(145, 2);
		ais->types.type6.dac1.dac1fid20.electrical	= UBITS(147, 2);
		ais->types.type6.dac1.dac1fid20.water		= UBITS(149, 2);
		ais->types.type6.dac1.dac1fid20.customs	= UBITS(151, 2);
		ais->types.type6.dac1.dac1fid20.cartage	= UBITS(153, 2);
		ais->types.type6.dac1.dac1fid20.crane		= UBITS(155, 2);
		ais->types.type6.dac1.dac1fid20.lift		= UBITS(157, 2);
		ais->types.type6.dac1.dac1fid20.medical	= UBITS(159, 2);
		ais->types.type6.dac1.dac1fid20.navrepair	= UBITS(161, 2);
		ais->types.type6.dac1.dac1fid20.provisions	= UBITS(163, 2);
		ais->types.type6.dac1.dac1fid20.shiprepair	= UBITS(165, 2);
		ais->types.type6.dac1.dac1fid20.surveyor	= UBITS(167, 2);
		ais->types.type6.dac1.dac1fid20.steam		= UBITS(169, 2);
		ais->types.type6.dac1.dac1fid20.tugs		= UBITS(171, 2);
		ais->types.type6.dac1.dac1fid20.solidwaste	= UBITS(173, 2);
		ais->types.type6.dac1.dac1fid20.liquidwaste	= UBITS(175, 2);
		ais->types.type6.dac1.dac1fid20.hazardouswaste	= UBITS(177, 2);
		ais->types.type6.dac1.dac1fid20.ballast	= UBITS(179, 2);
		ais->types.type6.dac1.dac1fid20.additional	= UBITS(181, 2);
		ais->types.type6.dac1.dac1fid20.regional1	= UBITS(183, 2);
		ais->types.type6.dac1.dac1fid20.regional2	= UBITS(185, 2);
		ais->types.type6.dac1.dac1fid20.future1	= UBITS(187, 2);
		ais->types.type6.dac1.dac1fid20.future2	= UBITS(189, 2);
		UCHARS(191, ais->types.type6.dac1.dac1fid20.berth_name);
		ais->types.type6.dac1.dac1fid20.berth_lon	= SBITS(311, 25);
		ais->types.type6.dac1.dac1fid20.berth_lat	= SBITS(336, 24);
		imo = true;
		break;
	    case 23:        /* IMO289 - Area notice - addressed */
		break;
	    case 25:	/* IMO289 - Dangerous cargo indication */
		ais->types.type6.dac1.dac1fid25.unit 	= UBITS(88, 2);
		ais->types.type6.dac1.dac1fid25.amount	= UBITS(90, 10);
		for (u = 0; 100 + u*17 < bitlen; u++) {
		    ais->types.type6.dac1.dac1fid25.cargos[u].code    = UBITS(100+u*17,4);
		    ais->types.type6.dac1.dac1fid25.cargos[u].subtype = UBITS(104+u*17,13);
		}
		ais->types.type6.dac1.dac1fid25.ncargos = u;
		imo = true;
		break;
	    case 28:	/* IMO289 - Route info - addressed */
/*
		ais->types.type6.dac1fid28.linkage	= UBITS(88, 10);
		ais->types.type6.dac1fid28.sender		= UBITS(98, 3);
		ais->types.type6.dac1fid28.rtype		= UBITS(101, 5);
		ais->types.type6.dac1fid28.month		= UBITS(106, 4);
		ais->types.type6.dac1fid28.day		= UBITS(110, 5);
		ais->types.type6.dac1fid28.hour		= UBITS(115, 5);
		ais->types.type6.dac1fid28.minute		= UBITS(120, 6);
		ais->types.type6.dac1fid28.duration	= UBITS(126, 18);
		ais->types.type6.dac1fid28.waycount	= UBITS(144, 5);
#define ARRAY_BASE 149
#define ELEMENT_SIZE 55
		for (u = 0; u < (uint8_t)ais->types.type6.dac1fid28.waycount; u++) {
		    int32_t a = ARRAY_BASE + (ELEMENT_SIZE*u);
		    ais->types.type6.dac1fid28.waypoints[u].lon = SBITS(a+0, 28);
		    ais->types.type6.dac1fid28.waypoints[u].lat = SBITS(a+28,27);
		}
#undef ARRAY_BASE
#undef ELEMENT_SIZE
		imo = true;
*/
		break;
	    case 30:	/* IMO289 - Text description - addressed */
		ais->types.type6.dac1.dac1fid30.linkage   = UBITS(88, 10);
		from_sixbit((int8_t *)bits,
			    98, bitlen-98,
			    ais->types.type6.dac1.dac1fid30.text);
		imo = true;
		break;
	    case 32:	/* IMO289 - Tidal Window */
		ais->types.type6.dac1.dac1fid32.month	= UBITS(88, 4);
		ais->types.type6.dac1.dac1fid32.day	= UBITS(92, 5);
#define ARRAY_BASE 97
#define ELEMENT_SIZE 88
		for (u = 0; ARRAY_BASE + (ELEMENT_SIZE*u) <= bitlen; u++) {
		    int32_t a = ARRAY_BASE + (ELEMENT_SIZE*u);
		    struct tidal_t *tp = &ais->types.type6.dac1.dac1fid32.tidals[u];
		    tp->lon	= SBITS(a + 0, 25);
		    tp->lat	= SBITS(a + 25, 24);
		    tp->from_hour	= UBITS(a + 49, 5);
		    tp->from_min	= UBITS(a + 54, 6);
		    tp->to_hour	= UBITS(a + 60, 5);
		    tp->to_min	= UBITS(a + 65, 6);
		    tp->cdir	= UBITS(a + 71, 9);
		    tp->cspeed	= UBITS(a + 80, 8);
		}
		ais->types.type6.dac1.dac1fid32.ntidals = u;
#undef ARRAY_BASE
#undef ELEMENT_SIZE
		imo = true;
		break;
	    }
	if (!imo)
	    (void)memcpy(ais->types.type6.dac1.bitdata,
			 (int8_t *)bits + (88 / BITS_PER_BYTE),
			 (ais->types.type6.bitcount + 7) / 8);
	break;
    case 7: /* Binary acknowledge */
    case 13: /* Safety Related Acknowledge */
    {
	uint32_t mmsi[4];
	if (bitlen < 72 || bitlen > 158) {
	    gpsd_report(LOG_WARN, "AIVDM message type %d size is out of range (%zd).\n",
			ais->type,
			bitlen);
	    return false;
	}
	for (u = 0; u < sizeof(mmsi)/sizeof(mmsi[0]); u++)
	    if (bitlen > 40 + 32*u)
		mmsi[u] = UBITS(40 + 32*u, 30);
	    else
		mmsi[u] = 0;
	/*@ -usedef @*/
	ais->types.type7.mmsi1 = mmsi[0];
	ais->types.type7.mmsi2 = mmsi[1];
	ais->types.type7.mmsi3 = mmsi[2];
	ais->types.type7.mmsi4 = mmsi[3];
	/*@ +usedef @*/
	break;
    }
    case 8: /* Binary Broadcast Message */
	if (bitlen < 56 || bitlen > 1008) {
	    gpsd_report(LOG_WARN, "AIVDM message type 8 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	//ais->types.type8.spare        = UBITS(38, 2);
	ais->types.type8.dac            = UBITS(40, 10);
	ais->types.type8.fid            = UBITS(50, 6);
	ais->types.type8.bitcount       = bitlen - 56;
	imo = false;
	if (ais->types.type8.dac == 1)
	    switch (ais->types.type8.fid) {
	    case 11:        /* IMO236 - Meteorological/Hydrological data */
		/* layout is almost identical to FID=31 from IMO289 */
		ais->types.type8.dac1.dac1fid11.lat		= SBITS(56, 24);
		ais->types.type8.dac1.dac1fid11.lon		= SBITS(80, 25);
		ais->types.type8.dac1.dac1fid11.day		= UBITS(105, 5);
		ais->types.type8.dac1.dac1fid11.hour		= UBITS(110, 5);
		ais->types.type8.dac1.dac1fid11.minute		= UBITS(115, 6);
		ais->types.type8.dac1.dac1fid11.wspeed		= UBITS(121, 7);
		ais->types.type8.dac1.dac1fid11.wgust		= UBITS(128, 7);
		ais->types.type8.dac1.dac1fid11.wdir		= UBITS(135, 9);
		ais->types.type8.dac1.dac1fid11.wgustdir	= UBITS(144, 9); 
		ais->types.type8.dac1.dac1fid11.airtemp	= UBITS(153, 11);
		ais->types.type8.dac1.dac1fid11.humidity	= UBITS(164, 7);
		ais->types.type8.dac1.dac1fid11.dewpoint	= UBITS(171, 10);
		ais->types.type8.dac1.dac1fid11.pressure	= UBITS(181, 9);
		ais->types.type8.dac1.dac1fid11.pressuretend	= UBITS(190, 2);
		ais->types.type8.dac1.dac1fid11.visibility	= UBITS(192, 8);
		ais->types.type8.dac1.dac1fid11.waterlevel	= UBITS(200, 9);
		ais->types.type8.dac1.dac1fid11.leveltrend	= UBITS(209, 2);
		ais->types.type8.dac1.dac1fid11.cspeed		= UBITS(211, 8);
		ais->types.type8.dac1.dac1fid11.cdir		= UBITS(219, 9);
		ais->types.type8.dac1.dac1fid11.cspeed2	= UBITS(228, 8);
		ais->types.type8.dac1.dac1fid11.cdir2		= UBITS(236, 9);
		ais->types.type8.dac1.dac1fid11.cdepth2	= UBITS(245, 5);
		ais->types.type8.dac1.dac1fid11.cspeed3	= UBITS(250, 8);
		ais->types.type8.dac1.dac1fid11.cdir3		= UBITS(258, 9);
		ais->types.type8.dac1.dac1fid11.cdepth3	= UBITS(267, 5);
		ais->types.type8.dac1.dac1fid11.waveheight	= UBITS(272, 8);
		ais->types.type8.dac1.dac1fid11.waveperiod	= UBITS(280, 6);
		ais->types.type8.dac1.dac1fid11.wavedir	= UBITS(286, 9);
		ais->types.type8.dac1.dac1fid11.swellheight	= UBITS(295, 8);
		ais->types.type8.dac1.dac1fid11.swellperiod	= UBITS(303, 6);
		ais->types.type8.dac1.dac1fid11.swelldir	= UBITS(309, 9);
		ais->types.type8.dac1.dac1fid11.seastate	= UBITS(318, 4);
		ais->types.type8.dac1.dac1fid11.watertemp	= UBITS(322, 10);
		ais->types.type8.dac1.dac1fid11.preciptype	= UBITS(332, 3);
		ais->types.type8.dac1.dac1fid11.salinity	= UBITS(335, 9);
		ais->types.type8.dac1.dac1fid11.ice		= UBITS(344, 2);
		imo = true;
		break;
	    case 13:        /* IMO236 - Fairway closed */
		UCHARS(56, ais->types.type8.dac1.dac1fid13.reason);
		UCHARS(176, ais->types.type8.dac1.dac1fid13.closefrom);
		UCHARS(296, ais->types.type8.dac1.dac1fid13.closeto);
		ais->types.type8.dac1.dac1fid13.radius 	= UBITS(416, 10);
		ais->types.type8.dac1.dac1fid13.extunit	= UBITS(426, 2);
		ais->types.type8.dac1.dac1fid13.fday   	= UBITS(428, 5);
		ais->types.type8.dac1.dac1fid13.fmonth 	= UBITS(433, 4);
		ais->types.type8.dac1.dac1fid13.fhour  	= UBITS(437, 5);
		ais->types.type8.dac1.dac1fid13.fminute	= UBITS(442, 6);
		ais->types.type8.dac1.dac1fid13.tday   	= UBITS(448, 5);
		ais->types.type8.dac1.dac1fid13.tmonth 	= UBITS(453, 4);
		ais->types.type8.dac1.dac1fid13.thour  	= UBITS(457, 5);
		ais->types.type8.dac1.dac1fid13.tminute	= UBITS(462, 6);
		/* skip 4 bits */
		imo = true;
		break;
	    case 15:        /* IMO236 - Extended ship and voyage */
		ais->types.type8.dac1.dac1fid15.airdraught	= UBITS(56, 11);
		/* skip 5 bits */
		imo = true;
		break;
	    case 17:        /* IMO289 - VTS-generated/synthetic targets */
#define ARRAY_BASE 56
#define ELEMENT_SIZE 122
		for (u = 0; ARRAY_BASE + (ELEMENT_SIZE*u) <= bitlen; u++) {
		    struct target_t *tp = &ais->types.type8.dac1.dac1fid17.targets[u];
		    int32_t a = ARRAY_BASE + (ELEMENT_SIZE*u);
		    tp->idtype = UBITS(a + 0, 2);
		    switch (tp->idtype) {
		    case DAC1FID17_IDTYPE_MMSI:
			tp->id.mmsi	= UBITS(a + 2, 42);
			break;
		    case DAC1FID17_IDTYPE_IMO:
			tp->id.imo	= UBITS(a + 2, 42);
			break;
		    case DAC1FID17_IDTYPE_CALLSIGN:
			UCHARS(a+2, tp->id.callsign);
			break;
		    default:
			UCHARS(a+2, tp->id.other);
			break;
		    }
		    /* skip 4 bits */
		    tp->lat	= SBITS(a + 48, 24);
		    tp->lon	= SBITS(a + 72, 25);
		    tp->course	= UBITS(a + 97, 9);
		    tp->second	= UBITS(a + 106, 6);
		    tp->speed	= UBITS(a + 112, 10);
		}
		ais->types.type8.dac1.dac1fid17.ntargets = u;
#undef ARRAY_BASE
#undef ELEMENT_SIZE
		imo = true;
		break;
	    case 19:        /* IMO289 - Marine Traffic Signal */
		ais->types.type8.dac1.dac1fid19.linkage	= UBITS(56, 10);
		UCHARS(66, ais->types.type8.dac1.dac1fid19.station);
		ais->types.type8.dac1.dac1fid19.lon	= SBITS(186, 25);
		ais->types.type8.dac1.dac1fid19.lat	= SBITS(211, 24);
		ais->types.type8.dac1.dac1fid19.status	= UBITS(235, 2);
		ais->types.type8.dac1.dac1fid19.signal	= UBITS(237, 5);
		ais->types.type8.dac1.dac1fid19.hour	= UBITS(242, 5);
		ais->types.type8.dac1.dac1fid19.minute	= UBITS(247, 6);
		ais->types.type8.dac1.dac1fid19.nextsignal	= UBITS(253, 5);
		/* skip 102 bits */
		imo = true;
		break;
	    case 21:        /* IMO289 - Weather obs. report from ship */
		break;
	    case 22:        /* IMO289 - Area notice - broadcast */
		break;
	    case 24:        /* IMO289 - Extended ship static & voyage-related data */
		break;
	    case 26:        /* IMO289 - Environmental */
		break;
	    case 27:        /* IMO289 - Route information - broadcast */
/*
		ais->types.type8.dac1.dac1fid27.linkage	= UBITS(56, 10);
		ais->types.type8.dac1.dac1fid27.sender	= UBITS(66, 3);
		ais->types.type8.dac1.dac1fid27.rtype	= UBITS(69, 5);
		ais->types.type8.dac1.dac1fid27.month	= UBITS(74, 4);
		ais->types.type8.dac1.dac1fid27.day	= UBITS(78, 5);
		ais->types.type8.dac1.dac1fid27.hour	= UBITS(83, 5);
		ais->types.type8.dac1.dac1fid27.minute	= UBITS(88, 6);
		ais->types.type8.dac1.dac1fid27.duration	= UBITS(94, 18);
		ais->types.type8.dac1.dac1fid27.waycount	= UBITS(112, 5);
#define ARRAY_BASE 117
#define ELEMENT_SIZE 55
		for (i = 0; i < ais->types.type8.dac1.dac1fid27.waycount; i++) {
		    int32_t a = ARRAY_BASE + (ELEMENT_SIZE*i);
		    ais->types.type8.dac1.dac1fid27.waypoints[i].lon	= SBITS(a + 0, 28);
		    ais->types.type8.dac1.dac1fid27.waypoints[i].lat	= SBITS(a + 28, 27);
		}
#undef ARRAY_BASE
#undef ELEMENT_SIZE
		imo = true;
*/
		break;
	    case 29:        /* IMO289 - Text Description - broadcast */
		ais->types.type8.dac1.dac1fid29.linkage   = UBITS(56, 10);
		from_sixbit((int8_t *)bits,
			    66, bitlen-66,
			    ais->types.type8.dac1.dac1fid29.text);
		imo = true;
		break;
	    case 31:        /* IMO289 - Meteorological/Hydrological data */
		ais->types.type8.dac1.dac1fid31.lon		= SBITS(56, 25);
		ais->types.type8.dac1.dac1fid31.lat		= SBITS(81, 24);
		ais->types.type8.dac1.dac1fid31.accuracy       = (bool)UBITS(105, 1);
		ais->types.type8.dac1.dac1fid31.day		= UBITS(106, 5);
		ais->types.type8.dac1.dac1fid31.hour		= UBITS(111, 5);
		ais->types.type8.dac1.dac1fid31.minute		= UBITS(116, 6);
		ais->types.type8.dac1.dac1fid31.wspeed		= UBITS(122, 7);
		ais->types.type8.dac1.dac1fid31.wgust		= UBITS(129, 7);
		ais->types.type8.dac1.dac1fid31.wdir		= UBITS(136, 9);
		ais->types.type8.dac1.dac1fid31.wgustdir	= UBITS(145, 9); 
		ais->types.type8.dac1.dac1fid31.airtemp	= SBITS(154, 11);
		ais->types.type8.dac1.dac1fid31.humidity	= UBITS(165, 7);
		ais->types.type8.dac1.dac1fid31.dewpoint	= SBITS(172, 10);
		ais->types.type8.dac1.dac1fid31.pressure	= UBITS(182, 9);
		ais->types.type8.dac1.dac1fid31.pressuretend	= UBITS(191, 2);
		ais->types.type8.dac1.dac1fid31.visgreater	= UBITS(193, 1);
		ais->types.type8.dac1.dac1fid31.visibility	= UBITS(194, 7);
		ais->types.type8.dac1.dac1fid31.waterlevel	= UBITS(201, 12);
		ais->types.type8.dac1.dac1fid31.leveltrend	= UBITS(213, 2);
		ais->types.type8.dac1.dac1fid31.cspeed		= UBITS(215, 8);
		ais->types.type8.dac1.dac1fid31.cdir		= UBITS(223, 9);
		ais->types.type8.dac1.dac1fid31.cspeed2	= UBITS(232, 8);
		ais->types.type8.dac1.dac1fid31.cdir2		= UBITS(240, 9);
		ais->types.type8.dac1.dac1fid31.cdepth2	= UBITS(249, 5);
		ais->types.type8.dac1.dac1fid31.cspeed3	= UBITS(254, 8);
		ais->types.type8.dac1.dac1fid31.cdir3		= UBITS(262, 9);
		ais->types.type8.dac1.dac1fid31.cdepth3	= UBITS(271, 5);
		ais->types.type8.dac1.dac1fid31.waveheight	= UBITS(276, 8);
		ais->types.type8.dac1.dac1fid31.waveperiod	= UBITS(284, 6);
		ais->types.type8.dac1.dac1fid31.wavedir	= UBITS(290, 9);
		ais->types.type8.dac1.dac1fid31.swellheight	= UBITS(299, 8);
		ais->types.type8.dac1.dac1fid31.swellperiod	= UBITS(307, 6);
		ais->types.type8.dac1.dac1fid31.swelldir	= UBITS(313, 9);
		ais->types.type8.dac1.dac1fid31.seastate	= UBITS(322, 4);
		ais->types.type8.dac1.dac1fid31.watertemp	= SBITS(326, 10);
		ais->types.type8.dac1.dac1fid31.preciptype	= UBITS(336, 3);
		ais->types.type8.dac1.dac1fid31.salinity	= UBITS(339, 9);
		ais->types.type8.dac1.dac1fid31.ice		= UBITS(348, 2);
		imo = true;
		break;
	    }
	/* land here if we failed to match a known DAC/FID */
	if (!imo)
	    (void)memcpy(ais->types.type8.dac1.bitdata,
			 (int8_t *)bits + (56 / BITS_PER_BYTE),
			 (ais->types.type8.bitcount + 7) / 8);
	break;
    case 9: /* Standard SAR Aircraft Position Report */
	if (bitlen != 168) {
	    gpsd_report(LOG_WARN, "AIVDM message type 9 size not 168 bits (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type9.alt		= UBITS(38, 12);
	ais->types.type9.speed		= UBITS(50, 10);
	ais->types.type9.accuracy		= (bool)UBITS(60, 1);
	ais->types.type9.lon		= SBITS(61, 28);
	ais->types.type9.lat		= SBITS(89, 27);
	ais->types.type9.course		= UBITS(116, 12);
	ais->types.type9.second		= UBITS(128, 6);
	ais->types.type9.regional		= UBITS(134, 8);
	ais->types.type9.dte		= UBITS(142, 1);
	//ais->types.type9.spare		= UBITS(143, 3);
	ais->types.type9.assigned		= UBITS(146, 1)!=0;
	ais->types.type9.raim		= UBITS(147, 1)!=0;
	ais->types.type9.radio		= UBITS(148, 19);
	break;
    case 10: /* UTC/Date inquiry */
	if (bitlen != 72) {
	    gpsd_report(LOG_WARN, "AIVDM message type 10 size not 72 bits (%zd).\n",
			bitlen);
	    return false;
	}
	//ais->types.type10.spare        = UBITS(38, 2);
	ais->types.type10.dest_mmsi      = UBITS(40, 30);
	//ais->types.type10.spare2       = UBITS(70, 2);
	break;
    case 12: /* Safety Related Message */
	if (bitlen < 72 || bitlen > 1008) {
	    gpsd_report(LOG_WARN, "AIVDM message type 12 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type12.seqno          = UBITS(38, 2);
	ais->types.type12.dest_mmsi      = UBITS(40, 30);
	ais->types.type12.retransmit     = (bool)UBITS(70, 1);
	//ais->types.type12.spare        = UBITS(71, 1);
	from_sixbit((int8_t *)bits,
		    72, bitlen-72,
		    ais->types.type12.text);
	break;
    case 14:	/* Safety Related Broadcast Message */
	if (bitlen < 40 || bitlen > 1008) {
	    gpsd_report(LOG_WARN, "AIVDM message type 14 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	//ais->types.type14.spare          = UBITS(38, 2);
	from_sixbit((int8_t *)bits,
		    40, bitlen-40,
		    ais->types.type14.text);
	break;
    case 15:	/* Interrogation */
	if (bitlen < 88 || bitlen > 168) {
	    gpsd_report(LOG_WARN, "AIVDM message type 15 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	(void)memset(&ais->types.type15, '\0', sizeof(ais->types.type15));
	//ais->types.type14.spare         = UBITS(38, 2);
	ais->types.type15.mmsi1		= UBITS(40, 30);
	ais->types.type15.type1_1		= UBITS(70, 6);
	ais->types.type15.type1_1		= UBITS(70, 6);
	ais->types.type15.offset1_1	= UBITS(76, 12);
	//ais->types.type14.spare2        = UBITS(88, 2);
	if (bitlen > 90) {
	    ais->types.type15.type1_2	= UBITS(90, 6);
	    ais->types.type15.offset1_2	= UBITS(96, 12);
	    //ais->types.type14.spare3    = UBITS(108, 2);
	    if (bitlen > 110) {
		ais->types.type15.mmsi2	= UBITS(110, 30);
		ais->types.type15.type2_1	= UBITS(140, 6);
		ais->types.type15.offset2_1	= UBITS(146, 12);
		//ais->types.type14.spare4	= UBITS(158, 2);
	    }
	}
	break;
    case 16:	/* Assigned Mode Command */
	if (bitlen != 96 && bitlen != 144) {
	    gpsd_report(LOG_WARN, "AIVDM message type 16 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type16.mmsi1		= UBITS(40, 30);
	ais->types.type16.offset1		= UBITS(70, 12);
	ais->types.type16.increment1	= UBITS(82, 10);
	if (bitlen < 144)
	    ais->types.type16.mmsi2=ais->types.type16.offset2=ais->types.type16.increment2 = 0;
	else {
	    ais->types.type16.mmsi2	= UBITS(92, 30);
	    ais->types.type16.offset2	= UBITS(122, 12);
	    ais->types.type16.increment2	= UBITS(134, 10);
	}
	break;
    case 17:	/* GNSS Broadcast Binary Message */
	if (bitlen < 80 || bitlen > 816) {
	    gpsd_report(LOG_WARN, "AIVDM message type 17 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	//ais->types.type17.spare         = UBITS(38, 2);
	ais->types.type17.lon		= UBITS(40, 18);
	ais->types.type17.lat		= UBITS(58, 17);
	//ais->types.type17.spare	        = UBITS(75, 4);
	ais->types.type17.bitcount        = bitlen - 80;
	(void)memcpy(ais->types.type17.bitdata,
		     (int8_t *)bits + (80 / BITS_PER_BYTE),
		     (ais->types.type17.bitcount + 7) / 8);
	break;
    case 18:	/* Standard Class B CS Position Report */
	if (bitlen != 168) {
	    gpsd_report(LOG_WARN, "AIVDM message type 18 size not 168 bits (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type18.reserved	= UBITS(38, 8);
	ais->types.type18.speed		= UBITS(46, 10);
	ais->types.type18.accuracy	= UBITS(56, 1)!=0;
	ais->types.type18.lon		= SBITS(57, 28);
	ais->types.type18.lat		= SBITS(85, 27);
	ais->types.type18.course		= UBITS(112, 12);
	ais->types.type18.heading		= UBITS(124, 9);
	ais->types.type18.second		= UBITS(133, 6);
	ais->types.type18.regional	= UBITS(139, 2);
	ais->types.type18.cs		= UBITS(141, 1)!=0;
	ais->types.type18.display 	= UBITS(142, 1)!=0;
	ais->types.type18.dsc     	= UBITS(143, 1)!=0;
	ais->types.type18.band    	= UBITS(144, 1)!=0;
	ais->types.type18.msg22   	= UBITS(145, 1)!=0;
	ais->types.type18.assigned	= UBITS(146, 1)!=0;
	ais->types.type18.raim		= UBITS(147, 1)!=0;
	ais->types.type18.radio		= UBITS(148, 20);
	break;
    case 19:	/* Extended Class B CS Position Report */
	if (bitlen != 312) {
	    gpsd_report(LOG_WARN, "AIVDM message type 19 size not 312 bits (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type19.reserved     = UBITS(38, 8);
	ais->types.type19.speed        = UBITS(46, 10);
	ais->types.type19.accuracy     = UBITS(56, 1)!=0;
	ais->types.type19.lon          = SBITS(57, 28);
	ais->types.type19.lat          = SBITS(85, 27);
	ais->types.type19.course       = UBITS(112, 12);
	ais->types.type19.heading      = UBITS(124, 9);
	ais->types.type19.second       = UBITS(133, 6);
	ais->types.type19.regional     = UBITS(139, 4);
	UCHARS(143, ais->types.type19.shipname);
	ais->types.type19.shiptype     = UBITS(263, 8);
	ais->types.type19.to_bow       = UBITS(271, 9);
	ais->types.type19.to_stern     = UBITS(280, 9);
	ais->types.type19.to_port      = UBITS(289, 6);
	ais->types.type19.to_starboard = UBITS(295, 6);
	ais->types.type19.epfd         = UBITS(299, 4);
	ais->types.type19.raim         = UBITS(302, 1)!=0;
	ais->types.type19.dte          = UBITS(305, 1)!=0;
	ais->types.type19.assigned     = UBITS(306, 1)!=0;
	//ais->types.type19.spare      = UBITS(307, 5);
	break;
    case 20:	/* Data Link Management Message */
	if (bitlen < 72 || bitlen > 160) {
	    gpsd_report(LOG_WARN, "AIVDM message type 20 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	//ais->types.type20.spare		= UBITS(38, 2);
	ais->types.type20.offset1		= UBITS(40, 12);
	ais->types.type20.number1		= UBITS(52, 4);
	ais->types.type20.timeout1	= UBITS(56, 3);
	ais->types.type20.increment1	= UBITS(59, 11);
	ais->types.type20.offset2		= UBITS(70, 12);
	ais->types.type20.number2		= UBITS(82, 4);
	ais->types.type20.timeout2	= UBITS(86, 3);
	ais->types.type20.increment2	= UBITS(89, 11);
	ais->types.type20.offset3		= UBITS(100, 12);
	ais->types.type20.number3		= UBITS(112, 4);
	ais->types.type20.timeout3	= UBITS(116, 3);
	ais->types.type20.increment3	= UBITS(119, 11);
	ais->types.type20.offset4		= UBITS(130, 12);
	ais->types.type20.number4		= UBITS(142, 4);
	ais->types.type20.timeout4	= UBITS(146, 3);
	ais->types.type20.increment4	= UBITS(149, 11);
	break;
    case 21:	/* Aid-to-Navigation Report */
	if (bitlen < 272 || bitlen > 360) {
	    gpsd_report(LOG_WARN, "AIVDM message type 21 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type21.aid_type = UBITS(38, 5);
	from_sixbit((int8_t *)bits,
		    43, 21, ais->types.type21.name);
	if (strlen(ais->types.type21.name) == 20 && bitlen > 272)
	    from_sixbit((int8_t *)bits,
			272, (bitlen - 272)/6,
			ais->types.type21.name+20);
	ais->types.type21.accuracy     = UBITS(163, 1);
	ais->types.type21.lon          = SBITS(164, 28);
	ais->types.type21.lat          = SBITS(192, 27);
	ais->types.type21.to_bow       = UBITS(219, 9);
	ais->types.type21.to_stern     = UBITS(228, 9);
	ais->types.type21.to_port      = UBITS(237, 6);
	ais->types.type21.to_starboard = UBITS(243, 6);
	ais->types.type21.epfd         = UBITS(249, 4);
	ais->types.type21.second       = UBITS(253, 6);
	ais->types.type21.off_position = UBITS(259, 1)!=0;
	ais->types.type21.regional     = UBITS(260, 8);
	ais->types.type21.raim         = UBITS(268, 1)!=0;
	ais->types.type21.virtual_aid  = UBITS(269, 1)!=0;
	ais->types.type21.assigned     = UBITS(270, 1)!=0;
	//ais->types.type21.spare      = UBITS(271, 1);
	break;
    case 22:	/* Channel Management */
	if (bitlen != 168) {
	    gpsd_report(LOG_WARN, "AIVDM message type 22 size not 168 bits (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type22.channel_a    = UBITS(40, 12);
	ais->types.type22.channel_b    = UBITS(52, 12);
	ais->types.type22.txrx         = UBITS(64, 4);
	ais->types.type22.power        = UBITS(68, 1);
	ais->types.type22.addressed    = UBITS(139, 1);
	if (!ais->types.type22.addressed) {
	    ais->types.type22.area_mmsi.area.ne_lon       = SBITS(69, 18);
	    ais->types.type22.area_mmsi.area.ne_lat       = SBITS(87, 17);
	    ais->types.type22.area_mmsi.area.sw_lon       = SBITS(104, 18);
	    ais->types.type22.area_mmsi.area.sw_lat       = SBITS(122, 17);
	} else {
	    ais->types.type22.area_mmsi.mmsi.dest1             = UBITS(69, 30);
	    ais->types.type22.area_mmsi.mmsi.dest2             = UBITS(104, 30);
	}
	ais->types.type22.band_a       = UBITS(140, 1);
	ais->types.type22.band_b       = UBITS(141, 1);
	ais->types.type22.zonesize     = UBITS(142, 3);
	break;
    case 23:	/* Group Assignment Command */
	if (bitlen != 160) {
	    gpsd_report(LOG_WARN, "AIVDM message type 23 size not 160 bits (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type23.ne_lon       = SBITS(40, 18);
	ais->types.type23.ne_lat       = SBITS(58, 17);
	ais->types.type23.sw_lon       = SBITS(75, 18);
	ais->types.type23.sw_lat       = SBITS(93, 17);
	ais->types.type23.stationtype  = UBITS(110, 4);
	ais->types.type23.shiptype     = UBITS(114, 8);
	ais->types.type23.txrx         = UBITS(144, 4);
	ais->types.type23.interval     = UBITS(146, 4);
	ais->types.type23.quiet        = UBITS(150, 4);
	break;
#if 0
    case 24:	/* Class B CS Static Data Report */
	switch (UBITS(38, 2)) {
	case 0:
	    if (bitlen != 160) {
		gpsd_report(LOG_WARN, "AIVDM message type 24A size not 160 bits (%zd).\n",
			    bitlen);
		return false;
	    }
	    /* save incoming 24A shipname/MMSI pairs in a circular queue */
	    {
		struct ais_type24a_t *saveptr = &type24_queue->ships[type24_queue->index];

		gpsd_report(LOG_PROG,
			    "AIVDM: 24A from %09u stashed.\n",
			    ais->mmsi);
		saveptr->mmsi = ais->mmsi;
		UCHARS(40, saveptr->shipname);
		++type24_queue->index;
		type24_queue->index %= MAX_TYPE24_INTERLEAVE;
	    }
	    //ais->types.type24.a.spare	= UBITS(160, 8);
	    return false;	/* data only partially decoded */
	case 1:
	    if (bitlen != 168) {
		gpsd_report(LOG_WARN, "AIVDM message type 24B size not 168 bits (%zd).\n",
			    bitlen);
		return false;
	    }
	    /* search the 24A queue for a matching MMSI */
	    for (i = 0; i < MAX_TYPE24_INTERLEAVE; i++) {
		if (type24_queue->ships[i].mmsi == ais->mmsi) {
		    (void)strlcpy(ais->types.type24.shipname,
				  type24_queue->ships[i].shipname,
				  sizeof(type24_queue->ships[i].shipname));
		    ais->types.type24.shiptype = UBITS(40, 8);
		    UCHARS(48, ais->types.type24.vendorid);
		    UCHARS(90, ais->types.type24.callsign);
		    if (AIS_AUXILIARY_MMSI(ais->mmsi)) {
			ais->types.type24.mothership_mmsi   = UBITS(132, 30);
		    } else {
			ais->types.type24.dim.to_bow        = UBITS(132, 9);
			ais->types.type24.dim.to_stern      = UBITS(141, 9);
			ais->types.type24.dim.to_port       = UBITS(150, 6);
			ais->types.type24.dim.to_starboard  = UBITS(156, 6);
		    }
		    //ais->types.type24.b.spare	    = UBITS(162, 8);
		    gpsd_report(LOG_PROG,
				"AIVDM 24B from %09u matches a 24A.\n",
				ais->mmsi);
		    /* prevent false match if a 24B is repeated */
		    type24_queue->ships[i].mmsi = 0;
		    return true;
		}
	    }
	    gpsd_report(LOG_WARN,
			"AIVDM 24B from %09u can't be matched to a 24A.\n",
			ais->mmsi);
	    return false;
	default:
	    gpsd_report(LOG_WARN, "AIVDM message type 24 of subtype unknown.\n");
	    return false;
	}
#endif

	// break;
    case 25:	/* Binary Message, Single Slot */
	/* this check and the following one reject line noise */
	if (bitlen < 40 || bitlen > 168) {
	    gpsd_report(LOG_WARN, "AIVDM message type 25 size not between 40 to 168 bits (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type25.addressed	= (bool)UBITS(38, 1);
	ais->types.type25.structured	= (bool)UBITS(39, 1);
	if (bitlen < (uint32_t)(40 + (16*ais->types.type25.structured) + (30*ais->types.type25.addressed))) {
	    gpsd_report(LOG_WARN, "AIVDM message type 25 too short for mode.\n");
	    return false;
	}
	if (ais->types.type25.addressed)
	    ais->types.type25.dest_mmsi   = UBITS(40, 30);
	if (ais->types.type25.structured)
	    ais->types.type25.app_id      = UBITS(40+ais->types.type25.addressed*30,16);
	/*
	 * Not possible to do this right without machinery we
	 * don't yet have.  The problem is that if the addressed
	 * bit is on, the bitfield start won't be on a byte
	 * boundary. Thus the formulas below (and in message type 26)
	 * will work perfectly for broadcast messages, but for addressed
	 * messages the retrieved data will be led by the 30 bits of
	 * the destination MMSI
	 */
	ais->types.type25.bitcount       = bitlen - 40 - 16*ais->types.type25.structured;
	(void)memcpy(ais->types.type25.bitdata,
		     (int8_t *)bits+5 + 2 * ais->types.type25.structured,
		     (ais->types.type25.bitcount + 7) / 8);
	break;
    case 26:	/* Binary Message, Multiple Slot */
	if (bitlen < 60 || bitlen > 1004) {
	    gpsd_report(LOG_WARN, "AIVDM message type 26 size is out of range (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type26.addressed	= (bool)UBITS(38, 1);
	ais->types.type26.structured	= (bool)UBITS(39, 1);
	if ((int32_t)bitlen < 40 + 16*ais->types.type26.structured + 30*ais->types.type26.addressed + 20) {
	    gpsd_report(LOG_WARN, "AIVDM message type 26 too short for mode.\n");
	    return false;
	}
	if (ais->types.type26.addressed)
	    ais->types.type26.dest_mmsi   = UBITS(40, 30);
	if (ais->types.type26.structured)
	    ais->types.type26.app_id      = UBITS(40+ais->types.type26.addressed*30,16);
	ais->types.type26.bitcount        = bitlen - 60 - 16*ais->types.type26.structured;
	(void)memcpy(ais->types.type26.bitdata,
		     (int8_t *)bits+5 + 2 * ais->types.type26.structured,
		     (ais->types.type26.bitcount + 7) / 8);
	break;
    case 27:	/* Long Range AIS Broadcast message */
	if (bitlen != 96) {
	    gpsd_report(LOG_WARN, "AIVDM message type 27 size not 96 bits (%zd).\n",
			bitlen);
	    return false;
	}
	ais->types.type27.accuracy        = (bool)UBITS(38, 1);
	ais->types.type27.raim		= UBITS(39, 1)!=0;
	ais->types.type27.status		= UBITS(40, 4);
	ais->types.type27.lon		= SBITS(44, 18);
	ais->types.type27.lat		= SBITS(62, 17);
	ais->types.type27.speed		= UBITS(79, 6);
	ais->types.type27.course		= UBITS(85, 9);
	ais->types.type27.gnss            = (bool)UBITS(94, 1);
	break;
    default:
	gpsd_report(LOG_ERROR, "Unparsed AIVDM message type %d.\n",ais->type);
	return false;
    }
    /* *INDENT-ON* */
#undef UCHARS
#undef SBITS
#undef UBITS
#undef BITS_PER_BYTE

    /* data is fully decoded */
    return true;
}

/*
 * Copy src to string dst of size siz.  At most siz-1 characters
 * will be copied.  Always NUL terminates (unless siz == 0).
 * Returns strlen(src); if retval >= siz, truncation occurred.
 */
/*@ -mayaliasunique -mustdefine @*/
/*size_t strlcpy(int8_t *dst, const int8_t *src, size_t siz)
{
    size_t len = strlen(src);
    if (siz != 0) {
    if (len >= siz) {
        memcpy(dst, src, siz - 1);
        dst[siz - 1] = '\0';
    } else
        memcpy(dst, src, len + 1);
    }
    return len;
}  */ 
/*@ -mayaliasunique -mustdefine @*/

/*
static uint8_t type5_buf[1024];
static size_t type5_bitlen = 0;
void add_type5_buf(uint8_t *buf, size_t bitlen)
{
	int32_t i=0, j, k;

	if(!type5_bitlen)
		memset(type5_buf, 0, 1024);

	while(i++ < bitlen)
	{
		k = i%8;
		j = i/8 + k ? 1:0;
		if((buf[j] >> (7 - k)) & 0x01)
			type5_buf[type5_bitlen/8 + (type5_bitlen%8) ? 1:0 ] |= (1 << (7 - type5_bitlen%8));
		type5_bitlen++;
	}
}
uint8_t *get_type5_buf(size_t *bitlen)
{
	*bitlen = type5_bitlen;
	return type5_buf;
}
void clear_type5_buf(void)
{
	type5_bitlen = 0;
	memset(type5_buf, 0, 1024);
}
*/

/*@ -charint @*/
/**************************************************************************
 *
 * AIVDM - ASCII armoring of binary AIS packets
 *
 **************************************************************************/

/*@ -fixedformalarray -usedef -branchstate @*/
static bool aivdm_decode(const int8_t *buf, size_t buflen,
		  struct aivdm_context_t ais_contexts[AIVDM_CHANNELS],
		  struct ais_t *ais)
		  /*int32_t debug)*/
{
#ifdef __UNUSED_DEBUG__
    int8_t *sixbits[64] = {
	"000000", "000001", "000010", "000011", "000100",
	"000101", "000110", "000111", "001000", "001001",
	"001010", "001011", "001100", "001101", "001110",
	"001111", "010000", "010001", "010010", "010011",
	"010100", "010101", "010110", "010111", "011000",
	"011001", "011010", "011011", "011100", "011101",
	"011110", "011111", "100000", "100001", "100010",
	"100011", "100100", "100101", "100110", "100111",
	"101000", "101001", "101010", "101011", "101100",
	"101101", "101110", "101111", "110000", "110001",
	"110010", "110011", "110100", "110101", "110110",
	"110111", "111000", "111001", "111010", "111011",
	"111100", "111101", "111110", "111111",
    };
#endif /* __UNUSED_DEBUG__ */
    int32_t nfrags, ifrag, nfields = 0;
    uint8_t *field[NMEA_MAX*2];
    uint8_t fieldcopy[NMEA_MAX*2+1];
    uint8_t *data, *cp;
    uint8_t ch, pad;
    struct aivdm_context_t *ais_context;
    int32_t i;

    if (buflen == 0)
	return false;

    /* we may need to dump the raw packet */
    gpsd_report(LOG_PROG, "AIVDM packet length %zd: %s\n", buflen, buf);

    /* first clear the result, making sure we don't return garbage */
    memset(ais, 0, sizeof(*ais));

    /* discard overlong sentences */
    if (strlen(buf) > sizeof(fieldcopy)-1) {
	gpsd_report(LOG_ERROR, "overlong AIVDM packet.\n");
	return false;
    }

    /* extract packet fields */
    (void)strlcpy((int8_t *)fieldcopy, buf, sizeof(fieldcopy));
    field[nfields++] = (uint8_t *)buf;
    for (cp = fieldcopy;
	 cp < fieldcopy + buflen; cp++)
	if (*cp == (uint8_t)',') {
	    *cp = '\0';
	    field[nfields++] = cp + 1;
	}

    /* discard sentences with exiguous commas; catches run-ons */
    if (nfields < 7) {
	gpsd_report(LOG_ERROR, "malformed AIVDM packet.\n");
	return false;
    }

    switch (field[4][0]) {
    /* FIXME: if fields[4] == "12", it doesn't detect the error */
    case '\0':
	/*
	 * Apparently an empty channel is normal for AIVDO sentences,
	 * which makes sense as they don't come in over radio.  This
	 * is going to break if there's ever an AIVDO type 24, though.
	 */
	if (strncmp((const int8_t *)field[0], "!AIVDO", 6) != 0)
	    gpsd_report(LOG_ERROR, "invalid empty AIS channel. Assuming 'A'\n");
	ais_context = &ais_contexts[0];
	break;
    case '1':
	gpsd_report(LOG_ERROR, "invalid AIS channel 0x%0x '%c'. Assuming 'A'\n",
		    field[4][0], (field[4][0] != (uint8_t)'\0' ? field[4][0]:' '));
	/*@fallthrough@*/
    case 'A':
	ais_context = &ais_contexts[0];
	break;
    case '2':
	gpsd_report(LOG_ERROR, "invalid AIS channel '2'. Assuming 'B'.\n");
	/*@fallthrough@*/
    case 'B':
	ais_context = &ais_contexts[1];
	break;
    default:
	gpsd_report(LOG_ERROR, "invalid AIS channel 0x%0X .\n", field[4][0]);
	return false;
    }

    nfrags = atoi((int8_t *)field[1]); /* number of fragments to expect */
    ifrag = atoi((int8_t *)field[2]); /* fragment id */
    data = field[5];
    pad = field[6][0]; /* number of padding bits */
    gpsd_report(LOG_PROG, "nfrags=%d, ifrag=%d, decoded_frags=%d, data=%s\n",
		nfrags, ifrag, ais_context->decoded_frags, data);

    /* assemble the binary data */

    /* check fragment ordering */
    if (ifrag != ais_context->decoded_frags + 1) {
	gpsd_report(LOG_ERROR, "invalid fragment #%d received, expected #%d.\n",
	                       ifrag, ais_context->decoded_frags + 1);
	/*ghye*/
/*	clear_type5_buf();
	ais_context->decoded_frags = 0;
	return false; */
#if 1
	if (ifrag != 1)
	    return false;
        /* else, ifrag==1: Just discard all that was previously decoded and
         * simply handle that packet */
        ais_context->decoded_frags = 0;
#endif
    }

    if (ifrag == 1) {
	(void)memset(ais_context->bits, '\0', sizeof(ais_context->bits));
	ais_context->bitlen = 0;
    }

	
    /* wacky 6-bit encoding, shades of FIELDATA */
    /*@ +charint @*/
    for (cp = data; cp < data + strlen((int8_t *)data); cp++) {
	ch = *cp;
	ch -= 48;
	if (ch >= 40)
	    ch -= 8;
#ifdef __UNUSED_DEBUG__
	gpsd_report(LOG_RAW, "%c: %s\n", *cp, sixbits[ch]);
#endif /* __UNUSED_DEBUG__ */
	/*@ -shiftnegative @*/
	for (i = 5; i >= 0; i--) {
	    if ((ch >> i) & 0x01) {
		ais_context->bits[ais_context->bitlen / 8] |=
		    (1 << (7 - ais_context->bitlen % 8));
	    }
	    ais_context->bitlen++;
	}
	/*@ +shiftnegative @*/
    }
    if (isdigit(pad))
	ais_context->bitlen -= (pad - '0');	/* ASCII assumption */
    /*@ -charint @*/

	/*ghye*/
/*	if(ifrag < nfrags)
	{
		add_type5_buf(ais_context->bits, ais_context->bitlen);
		ais_context->decoded_frags++;
		return false;
	}
	else if(ifrag != 1)
	{
		uint8_t *data;
		size_t bitlen;
		add_type5_buf(ais_context->bits, ais_context->bitlen);
		data = get_type5_buf(&bitlen);
		memcpy(ais_context->bits, data, bitlen/8 + (bitlen%8) ? 1:0);
		ais_context->bitlen = bitlen;
		ais_context->decoded_frags = 0;
		clear_type5_buf();
int32_t i;
for(i=0;i<bitlen/8;i++)
printf("%d ",ais_context->bits[i]);
printf("\n");
	}*/

    /* time to pass buffered-up data to where it's actually processed? */
    if (ifrag == nfrags) {
#if 0
	if (debug >= LOG_INF) { 
	    size_t clen = (ais_context->bitlen + 7) / 8;
	    gpsd_report(LOG_INF, "AIVDM payload is %zd bits, %zd chars: %s\n",
			ais_context->bitlen, clen,
			gpsd_hexdump((int8_t *)ais_context->bits, clen));
	}
#endif

        /* clear waiting fragments count */
        ais_context->decoded_frags = 0;

	/* decode the assembled binary packet */
	return ais_binary_decode(ais,
				 ais_context->bits,
				 ais_context->bitlen);
				 /*&ais_context->type24_queue);*/
    }

    /* we're still waiting on another sentence */
    ais_context->decoded_frags++;
    return false;
}

uint64_t /*gps_mask_t*/ aivdm_analyze(struct gps_device_t *session)
{
    if (session->packet.type == AIVDM_PACKET) {
	if (aivdm_decode
	    ((int8_t *)session->packet.outbuffer, session->packet.outbuflen,
	     session->aivdm, &session->gpsdata.ais/*, session->context->debug*/)) {
	    return ONLINE_SET | AIS_SET;
	} else
	    return ONLINE_SET;
//#ifdef NMEA_ENABLE
    } else if (session->packet.type == NMEA_PACKET) {
	return nmea_parse((int8_t *)session->packet.outbuffer, session);
//#endif /* NMEA_ENABLE */
    } else
	return 0;
}
