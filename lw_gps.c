
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "lw_gps.h"
#include "lw_stm32_uart.h"

#define GPS_MAX_MSG_LEN 128
#define GPS_MAX_ARRY 20

struct gps_info_{
	struct{
		uint8_t rbuf[GPS_MAX_ARRY][GPS_MAX_MSG_LEN];
		uint8_t rbuf_arry_head;
		uint8_t rbuf_arry_tail;
		uint16_t rbuf_index[GPS_MAX_ARRY];
	}rbuf_info;
};
struct gps_info_ gps_info;

void lw_gps_init(void)
{
	com_para_t com_para;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	com_para.baudrate = 9600;
	com_para.data_9bit = false;
	com_para.parity = 0;
	com_para.port = 2;
	com_para.twoStopBits = false;
	com_init(&com_para);

}

void lw_gps_param_init(void)
{
	memset(&gps_info, 0, sizeof(gps_info));
}

static void lw_gps_clean_rbuf(void)
{
	memset(gps_info.rbuf_info.rbuf[gps_info.rbuf_info.rbuf_arry_head], 0, GPS_MAX_MSG_LEN);
	gps_info.rbuf_info.rbuf_index[gps_info.rbuf_info.rbuf_arry_head++] = 0;
	if(gps_info.rbuf_info.rbuf_arry_head >= GPS_MAX_ARRY)
		gps_info.rbuf_info.rbuf_arry_head = 0;


/*	gps_info.rbuf_info.rbuf_index = 0;
	memset(gps_info.rbuf_info.rbuf, 0, sizeof(gps_info.rbuf_info.rbuf)/sizeof(gps_info.rbuf_info.rbuf[0]));*/
}

int32_t lw_gps_get_rbuf(uint8_t *buf)
{
	int32_t ret = -1;
	
	if(gps_info.rbuf_info.rbuf_arry_head != gps_info.rbuf_info.rbuf_arry_tail)
	{
		memcpy(buf, gps_info.rbuf_info.rbuf[gps_info.rbuf_info.rbuf_arry_head],
			gps_info.rbuf_info.rbuf_index[gps_info.rbuf_info.rbuf_arry_head]);
		//p = gps_info.rbuf_info.rbuf[gps_info.rbuf_info.rbuf_arry_head];
		lw_gps_clean_rbuf();
		ret = 0;
	}
	return ret;
}

static uint8_t irq_data;
static void lw_gps_put_rbuf(uint8_t val)
{
	if(val == '$')
	{
		irq_data = gps_info.rbuf_info.rbuf_arry_tail;
		++ gps_info.rbuf_info.rbuf_arry_tail;
		if(gps_info.rbuf_info.rbuf_arry_tail >= GPS_MAX_ARRY)
			gps_info.rbuf_info.rbuf_arry_tail = 0;	
		if(gps_info.rbuf_info.rbuf_arry_tail == gps_info.rbuf_info.rbuf_arry_head)
		{
			gps_info.rbuf_info.rbuf_arry_tail = irq_data;
			gps_info.rbuf_info.rbuf_index[gps_info.rbuf_info.rbuf_arry_tail] = 0;
		}
	}
	gps_info.rbuf_info.rbuf[gps_info.rbuf_info.rbuf_arry_tail][gps_info.rbuf_info.rbuf_index[gps_info.rbuf_info.rbuf_arry_tail]] = val;		
	gps_info.rbuf_info.rbuf_index[gps_info.rbuf_info.rbuf_arry_tail]++;
}

int32_t lw_gps_recv(uint8_t val)
{
	//gps_info.rbuf_info.rbuf[gps_info.rbuf_info.rbuf_index ++] = val;
	lw_gps_put_rbuf(val);
}

int32_t lw_gps_send()
{}

#if 1
uint8_t testgpsbuf[GPS_MAX_MSG_LEN];
#include "./ais/proto.h"
uint8_t *lw_gps_get_result_buf(void)
{
	return testgpsbuf;
}
int32_t lw_gps_test(void)
{
	int32_t ret = -1;
/*
	int32_t nmea_ret;
	int32_t nmea_buf_len = strlen(testgpsbuf);
	struct gps_device_t session;
	int8_t * nmea_buf = testgpsbuf;
*/
	memset(testgpsbuf, 0, GPS_MAX_MSG_LEN);
	ret = lw_gps_get_rbuf(testgpsbuf);
	if(0 == ret)
	{
/*		com_send_message(1, "gps:");
		com_send_message(1, testgpsbuf);*/

#if 0
		session.packet.type = NMEA_PACKET;
		memcpy(session.packet.outbuffer, nmea_buf, nmea_buf_len);
		session.packet.outbuffer[nmea_buf_len] = '\0';
		session.packet.outbuflen = nmea_buf_len;		
		nmea_ret = aivdm_analyze(&session);

		if(nmea_ret)
		{
			switch(session.nmea.type)
			{
				case TYPE_GPRMC:
				printf("GPRMC:\n");
				lw_gprs_send_C_cmd(double lat, double lon, double speed, double track/*航向*/, bool isvalid /*数据状态: 1:有效，0:无效*/)
				break;
				default:
				break;	
					double speed;	/*Speed over ground, knots*/
			double track;	/*Track made good, degrees true*/
			double magnetic;	/*Magnetic Variation, degrees*/
		
			}
		}
#endif
	}

	return ret;
}
#endif
