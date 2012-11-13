#include "projects_conf.h"

#if defined(STM_SHIP)
#include "stm32f10x_rcc.h"
#include "string.h"

#include "lw_ais.h"
#include "lw_stm32_uart.h"
#include "proto.h"
#include "util.h"

/*
	接收AIS，再处理之并存放在另一buffer，通过GPRS发送
*/


struct ais_info_ ais_info;

static struct gps_device_t session;

void ais_rx_init(void)
{
	com_para_t com_para;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	com_para.baudrate = 38400;
	com_para.data_9bit = false;
	com_para.parity = 0;
	com_para.port = 3;
	com_para.twoStopBits = false;
	com_init(&com_para);
}
void ais_global_init(void)
{
	ais_info.raw_info.tail = 0;
	ais_info.parsed_info.head = 0;
	ais_info.parsed_info.parsed_cnt = 0;
	ais_info.parsed_info.tmp_cnt = 0;

	memset(&session, 0, sizeof(session));
}

void ais_parse_init(void)
{
	//ais_info.parsed_info.parsed_cnt = 0;
	//memset(ais_info.parsed_info.parsed, '\0', AIS_PARSED_MAX_LEN);
}

/*
针对AIS解析。查找首尾符合的位置
*/
static int8_t memstrht_ais(uint8_t *p, uint32_t len, uint8_t *phead, const uint32_t hlen, uint8_t *ptail, const uint32_t tlen, uint8_t **hret, uint32_t *lenret)
{
    uint32_t tmp;
    uint8_t *p1;
    uint8_t *p2;

    do {
        p1 = memstr(p, len, phead, hlen);
        if (!p1) {
            //return 1;

            /*考虑结尾仅有!AIVDM的前一部分，例如只有“!” */
            p1 = memstr(p, len, phead, 1);
            if (!p1) {
                return 1;
            }
            *hret = p1;
            return 2;
        }
        tmp = 0;
        do {
            p2 = memstr(p1+hlen +tmp, len - (p1 - p - hlen) - tmp, ptail, tlen);
            if (!p2) {
                *hret = p1;
                return 2;
            }
            tmp = p2-(p1+hlen)+tlen;
        }while((*(p2-1) <'0') || (*(p2-1) >'5') || (*(p2-2)!=',')); /*例如: ",0*3c" */

        len -= (p1+hlen-p);
        p = p1 + hlen;
    }while(p2+tlen - p1 >= 90); /*一个AIS语句82bytes*/

    *hret = p1;
    if (p2+1+2 > p+len) { /* *后面有两个字符 */
        return 1;
    }
    *lenret = p2+tlen - p1 +2; /* *后面有两个字符 */

    return 0;
}
uint8_t testais[64];
static void aivdm_proc(const uint8_t *buf, uint32_t len)
{
	int ret;
	
    session.packet.type = AIVDM_PACKET;
    memcpy(session.packet.outbuffer, buf, len);/*待解析的可见字符串*/
    session.packet.outbuflen = len;/*长度*/
#if 1
    ret = aivdm_analyze(&session);

    /*查看*/
    //sprintf(testais, "ais type:%d \n", session.gpsdata.ais.type);com_send_message(1, testais);
    if(ret & AIS_SET) {
    switch (session.gpsdata.ais.type)
    {   
    #if 0
        case 1:
        sprintf(testais, "ais UTC:%d\n", session.gpsdata.ais.types.type1.second);com_send_message(1, testais);
        sprintf(testais, "ais MMSI:%d\n", session.gpsdata.ais.mmsi);com_send_message(1, testais);
        sprintf(testais, "ais Latitude:%d,o:%d\n", session.gpsdata.ais.types.type1.lat /600000, session.gpsdata.ais.types.type1.lat);com_send_message(1, testais);
        sprintf(testais, "ais Longitude:%d,o:%d\n", session.gpsdata.ais.types.type1.lon /600000, session.gpsdata.ais.types.type1.lon);com_send_message(1, testais);
        //printf("ais Speed:%d\n", session.gpsdata.ais.type1.speed /10);
        sprintf(testais, "ais Speed:%d\n", session.gpsdata.ais.types.type1.speed);com_send_message(1, testais);
        sprintf(testais, "ais Heading:%d\n", session.gpsdata.ais.types.type1.heading);com_send_message(1, testais);
        sprintf(testais, "ais Course over ground:%f\n", ((double)(session.gpsdata.ais.types.type1.course)) /10 +\
                                                (session.gpsdata.ais.types.type1.course &0x01) * (float)0.1 );com_send_message(1, testais);
        sprintf(testais, "ais Rate of turn:%d\n", session.gpsdata.ais.types.type1.turn);com_send_message(1, testais);
        sprintf(testais, "ais Navigational status:%d\n", session.gpsdata.ais.types.type1.status);com_send_message(1, testais);
        /*printf("ais Nearest place:%d\n", session.gpsdata.type1.);*/
        break;
        case 2:
        sprintf(testais, "ais UTC:%d\n", session.gpsdata.ais.types.type1.second);com_send_message(1, testais);
        sprintf(testais, "ais MMSI:%d\n", session.gpsdata.ais.mmsi);com_send_message(1, testais);
        sprintf(testais, "ais Latitude:%d,o:%d\n", session.gpsdata.ais.types.type1.lat /600000, session.gpsdata.ais.types.type1.lat);com_send_message(1, testais);
        sprintf(testais, "ais Longitude:%d,o:%d\n", session.gpsdata.ais.types.type1.lon /600000, session.gpsdata.ais.types.type1.lon);com_send_message(1, testais);
        sprintf(testais, "ais Speed:%d\n", session.gpsdata.ais.types.type1.speed );com_send_message(1, testais);
        sprintf(testais, "ais Heading:%d\n", session.gpsdata.ais.types.type1.heading);com_send_message(1, testais);
        sprintf(testais, "ais Course over ground:%f\n", ((double)(session.gpsdata.ais.types.type1.course)) /10 +\
                                                (session.gpsdata.ais.types.type1.course &0x01) * (float)0.1 );com_send_message(1, testais);
        sprintf(testais, "ais Rate of turn:%d\n", session.gpsdata.ais.types.type1.turn);com_send_message(1, testais);
        sprintf(testais, "ais Navigational status:%d\n", session.gpsdata.ais.types.type1.status);com_send_message(1, testais);
        /*printf("ais Nearest place:%d\n", session.gpsdata.type1.);*/
        break;
        case 3:
        sprintf(testais, "ais UTC:%d\n", session.gpsdata.ais.types.type1.second);com_send_message(1, testais);
        sprintf(testais, "ais MMSI:%d\n", session.gpsdata.ais.mmsi);com_send_message(1, testais);
        sprintf(testais, "ais Latitude:%d,o:%d\n", session.gpsdata.ais.types.type1.lat /600000, session.gpsdata.ais.types.type1.lat);com_send_message(1, testais);
        sprintf(testais, "ais Longitude:%d,o:%d\n", session.gpsdata.ais.types.type1.lon /600000, session.gpsdata.ais.types.type1.lon);com_send_message(1, testais);
        sprintf(testais, "ais Speed:%d\n", session.gpsdata.ais.types.type1.speed );com_send_message(1, testais);
        sprintf(testais, "ais Heading:%d\n", session.gpsdata.ais.types.type1.heading);com_send_message(1, testais);
        sprintf(testais, "ais Course over ground:%f\n", ((double)(session.gpsdata.ais.types.type1.course)) /10 +\
                                                (session.gpsdata.ais.types.type1.course &0x01) * (float)0.1 );com_send_message(1, testais);
        sprintf(testais, "ais Rate of turn:%d\n", session.gpsdata.ais.types.type1.turn);com_send_message(1, testais);
        sprintf(testais, "ais Navigational status:%d\n", session.gpsdata.ais.types.type1.status);com_send_message(1, testais);
        /*printf("ais Nearest place:%d\n", session.gpsdata.type1.);*/
        break;
        case 5:
        sprintf(testais, "ais version:%d\n", session.gpsdata.ais.types.type5.ais_version);com_send_message(1, testais);
        sprintf(testais, "ais MMSI:%d\n", session.gpsdata.ais.mmsi);com_send_message(1, testais);
        sprintf(testais, "ais imo:%d\n", session.gpsdata.ais.types.type5.imo);com_send_message(1, testais);
        sprintf(testais, "ais shiptype:%d\n", session.gpsdata.ais.types.type5.shiptype);com_send_message(1, testais);
        sprintf(testais, "ais to_bow:%d\n", session.gpsdata.ais.types.type5.to_bow);com_send_message(1, testais);
        sprintf(testais, "ais to_stern:%d\n", session.gpsdata.ais.types.type5.to_stern);com_send_message(1, testais);
        sprintf(testais, "ais to_port:%d\n", session.gpsdata.ais.types.type5.to_port);com_send_message(1, testais);
        sprintf(testais, "ais to_starboard:%d\n", session.gpsdata.ais.types.type5.to_starboard);com_send_message(1, testais);
        sprintf(testais, "ais epfd:%d\n", session.gpsdata.ais.types.type5.epfd);com_send_message(1, testais);
        sprintf(testais, "ais time:%d:%d:%d:%d\n", session.gpsdata.ais.types.type5.month,session.gpsdata.ais.types.type5.day,
                                        session.gpsdata.ais.types.type5.hour,session.gpsdata.ais.types.type5.minute);com_send_message(1, testais);
        sprintf(testais, "ais draught:%d\n", session.gpsdata.ais.types.type5.draught);com_send_message(1, testais);
        sprintf(testais, "ais dte:%d\n", session.gpsdata.ais.types.type5.dte);com_send_message(1, testais);
        sprintf(testais, "ais callsign:%s\n", session.gpsdata.ais.types.type5.callsign);com_send_message(1, testais);
        sprintf(testais, "ais shipname:%s\n", session.gpsdata.ais.types.type5.shipname);com_send_message(1, testais);
        sprintf(testais, "ais destination:%s\n", session.gpsdata.ais.types.type5.destination);com_send_message(1, testais);
        break;
		#endif
    }
	
	test_save_ais2sd(buf, len);
    }
#endif
}

bool ais_raw_buf_conflict(void)
{
	uint8_t head;
	uint8_t tail;

	head = ais_info.parsed_info.head;
	tail = ais_info.raw_info.tail;
	tail = tail ? (tail -1): (AIS_RAW_MAX_NUM-1);
	return ((head == tail) ? true:false);
}

static uint8_t aisbuf[AIS_RAW_MAX_LEN<<1];
void ais_seqed_proc(void)
{
    int8_t ret;
    uint32_t len;
    uint8_t phead[]="!AIVDM";
    uint8_t ptail[]="*";
    uint8_t *p;
    uint32_t lenret;
    uint8_t *hret;

	/*FIXME:　可能需暂时关闭DMA中断*/
	
	/*不可处理正在接收的buf*/
	if (ais_raw_buf_conflict())
		return;

    if (ais_info.parsed_info.tmp_cnt > AIS_PARSED_TMP_MAX_LEN) {
        ais_info.parsed_info.tmp_cnt = AIS_PARSED_TMP_MAX_LEN;
    }
	//com_send_nchar(1,ais_info.raw_info.raw[ais_info.parsed_info.head], AIS_RAW_MAX_LEN);
    memcpy(aisbuf, ais_info.parsed_info.tmp, ais_info.parsed_info.tmp_cnt);
    memcpy(aisbuf +ais_info.parsed_info.tmp_cnt, ais_info.raw_info.raw[ais_info.parsed_info.head++], AIS_RAW_MAX_LEN);
    if (ais_info.parsed_info.head >= AIS_RAW_MAX_NUM)
        ais_info.parsed_info.head = 0;
    p = aisbuf;
    len = ais_info.parsed_info.tmp_cnt + AIS_RAW_MAX_LEN;
    while (1) {
        ret = memstrht_ais(p, len, phead, strlen(phead), ptail, strlen(ptail), &hret, &lenret);
        if (!ret) { /*found*/
            len -= hret+lenret - p;
            p = hret+lenret;
            /*printf("[parsed:] ");
            int i;
            for (i=0; i<lenret; i++)
                printf("%c",hret[i]);
            printf("\n");*/
            aivdm_proc(hret, lenret);
        }
        else if (1 == ret) {
            ais_info.parsed_info.tmp_cnt = 0;
            return ;
        }
        else { /*ret==2*/
            ais_info.parsed_info.tmp_cnt = ais_info.parsed_info.tmp_cnt + AIS_RAW_MAX_LEN - (hret- aisbuf);
            memcpy(ais_info.parsed_info.tmp, hret +
            ((ais_info.parsed_info.tmp_cnt > AIS_RAW_MAX_LEN) ? (ais_info.parsed_info.tmp_cnt - AIS_RAW_MAX_LEN):0), /*考虑剩余的不能超过AIS_RAW_MAX_LEN*/
                    ais_info.parsed_info.tmp_cnt - ((ais_info.parsed_info.tmp_cnt > AIS_RAW_MAX_LEN) ? (ais_info.parsed_info.tmp_cnt - AIS_RAW_MAX_LEN):0));
            ais_info.parsed_info.tmp_cnt -= ((ais_info.parsed_info.tmp_cnt > AIS_RAW_MAX_LEN) ? (ais_info.parsed_info.tmp_cnt - AIS_RAW_MAX_LEN):0);
            return ;
        }
    }
}

#endif

