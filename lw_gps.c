#include <string.h>

#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "lw_gps.h"
#include "lw_nmea.h"
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

#define GPS_PWR_ON()	GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define GPS_PWR_OFF()	GPIO_ResetBits(GPIOA, GPIO_Pin_1)

void lw_gps_init(void)
{
	extern volatile unsigned int Timer1;
	uint32_t i;
	
	com_para_t com_para;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	com_para.baudrate = 9600;
	com_para.data_9bit = false;
	com_para.parity = 0;
	com_para.port = 2;
	com_para.twoStopBits = false;
	com_init(&com_para);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPS_PWR_OFF();
	Timer1 = 300;
	while (Timer1) ;
	GPS_PWR_ON();

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
	if (gps_info.rbuf_info.rbuf_index[gps_info.rbuf_info.rbuf_arry_tail] < GPS_MAX_MSG_LEN -1)
		gps_info.rbuf_info.rbuf_index[gps_info.rbuf_info.rbuf_arry_tail]++;
}

void lw_gps_recv(uint8_t val)
{
	//gps_info.rbuf_info.rbuf[gps_info.rbuf_info.rbuf_index ++] = val;
	lw_gps_put_rbuf(val);
}

#if 1
uint8_t testgpsbuf[GPS_MAX_MSG_LEN];
#include "./ais/proto.h"
uint8_t *lw_get_gps_buf(void)
{
	return testgpsbuf;
}

static void reset_gps(void)
{
	extern volatile unsigned int Timer1;
	#define RESET_GPS_CMD "\x00D\x00A$PSRF101,0,0,0,0,0,0,12,4*10\x00D\x00A"
	com_send_string(2, RESET_GPS_CMD);
	GPS_PWR_OFF();
	Timer1 = 300;
	while (Timer1) ;
	GPS_PWR_ON();
}

int32_t lw_get_gps_sentence(void)
{
	extern volatile unsigned int check_gps_signal;
	int32_t ret = -1;
/*
	int32_t nmea_ret;
	int32_t nmea_buf_len = strlen(testgpsbuf);
	struct gps_device_t session;
	int8_t * nmea_buf = testgpsbuf;
*/

	if (!check_gps_signal) {
		reset_gps();
		check_gps_signal = 6000;
	}
	
	memset(testgpsbuf, '\0', GPS_MAX_MSG_LEN);
	ret = lw_gps_get_rbuf(testgpsbuf);
	if(0 == ret)
	{
		check_gps_signal = 6000;
/*
#define TESTRMC "$GPRMC,021115.000,A,2306.2713,N,11326.3310,E,0.00,,301012,,,D*74"
memcpy(testgpsbuf, TESTRMC, strlen(TESTRMC));*/

/*		com_send_message(1, "gps:");
		com_send_message(1, testgpsbuf);
*/

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

void  lw_gps_parse(void)
{
	uint8_t cnt = 0;
	
	while (lw_get_gps_sentence() == 0) {
		if (1 == lw_nmea_parse())
			break;
		if (cnt++ > 40)
			break;
	}
}
#endif

