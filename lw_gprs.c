#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "stm32f10x.h" //"stm32f10x_nvic.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "lw_gprs.h"
#include "lw_stm32_uart.h"

#define GPRS_MAX_MSG_LEN 512 //64
#define GPS_MAX_MSG_LEN 0xff

#define M_RST GPIO_Pin_8
#define M_ON GPIO_Pin_9
#define MG323_RESET() 	GPIO_SetBits(GPIOC, M_RST) /* 硬件复位>= 10ms */
#define MG323_NORM() GPIO_ResetBits(GPIOC, M_RST)
#define MG323_POWERSWITCH() GPIO_SetBits(GPIOC, M_ON)	/*开/关切换>=1s*/
#define MG323_POWERNORM() GPIO_ResetBits(GPIOC, M_ON)

#define ATE0 "ATE0\x00D\x00A"
#define CHECK_COPN "AT+COPN=?"
#define GET_CONPN "AT+COPN"
#define CHECK_COPS "AT+COPS=?"
#define GET_REGESTERED_COPS "AT+COPS?"
#define SET_CREG "AT+CREG=0" /* 8.3.3 */

#define CHECK_SICS "AT^SICS=?\x00D\x00A"	/* 10.1.1 */
#define SET_NETTYPE "AT^SICS=0,conType,GPRS0\x00D\x00A"	/* 10.1.3 */
#define SET_APNTYPE "AT^SICS=0,apn,3GNET\x00D\x00A"
#define SET_SRVTYPE "AT^SISS=0,srvType,Socket\x00D\x00A"
#define SET_CONID "AT^SISS=0,conId,0\x00D\x00A"
//#define SET_SRV_URL_1 "AT^SISS=0,address,\"socktcp://113.105.139.109:6969\"\x00D\x00A"
#define SET_SRV_URL_1 "AT^SISS=0,address,\"socktcp://203.88.202.116:7788\"\x00D\x00A"
#define OPEN_CON0 "AT^SISO=0\x00D\x00A"
#define CLOSE_CON0 "AT^SISC=0\x00D\x00A"
#define GET_IMEI "AT+CGSN\x00D\x00A"
#define MG323_TP_TRANS "AT^IPENTRANS=0\x00D\x00A"

#define GPRS_RXNE_IRQ_ENABLE() USART_ITConfig(USART1, USART_IT_RXNE, ENABLE)
#define GPRS_RXNE_IRQ_DISABLE() USART_ITConfig(USART1, USART_IT_RXNE, DISABLE)

typedef enum{
	GPRS_STATUS_NOINIT,
	GRPS_STATUS_START,	/* 等待GPRS模块发送^SYSSTART */
	GPRS_STATUS_ATE,
	GPRS_STATUS_GET_IMEI,
	GPRS_STATUS_NETTYPE,
	GPRS_STATUS_APNTYPE,
	GPRS_STATUS_SRVTYPE,
	GPRS_STATUS_CONID,
	GPRS_STATUS_SRVURL,
	GPRS_STATUS_SOCKET_OPEN,
	GPRS_STATUS_SOCKET_TP,
	GPRS_STATUS_SOCKET_TP_FINISH,
	GPRS_STATUS_SOCKET_WRITE,
	GPRS_STATUS_SOCKET_WRITE_MSG,
	GPRS_STATUS_SOCKET_WILL_CLOSED,
	GPRS_STATUS_SOCKET_READ,
	GPRS_STATUS_SOCKET_CLOSE,

	GPRS_STATUS_MAX,
}GPRS_MODULE_STATUS;

struct _gprs_info{
	bool ate0;
	uint8_t imei[15];
	GPRS_MODULE_STATUS gprs_status;
	struct {
		bool in_tp_mode;
		bool quit_socket;
		bool sec_msg;	/* 第二次发送数据 */
		bool socket_opend;
		bool socket_closed_by_srv;
		uint32_t session_max_len;
	}send_status;
	struct {
		uint8_t rbuf[GPRS_MAX_MSG_LEN];
		/*uint8_t*/ uint16_t rbuf_index;
	}rbuf_info;
	uint32_t tmpbuf_len;
	uint32_t tmpbuf2_len;
	uint8_t tmpbuf[GPRS_MAX_MSG_LEN];
	uint8_t tmpbuf2[GPRS_MAX_MSG_LEN];
	struct {
		uint8_t sbuf[GPRS_MAX_MSG_LEN];
		uint8_t sbuf_len;
		uint8_t gps_sbuf[GPS_MAX_MSG_LEN];
		uint8_t gps_sbuf_len;
	}sbuf_info;
};
struct _gprs_info gprs_info;

extern volatile unsigned int Timer1, Timer2;

static void lw_clean_gprs_sbuf(void)
{
	gprs_info.sbuf_info.sbuf_len = 0;
	memset(gprs_info.sbuf_info.sbuf, 0, GPRS_MAX_MSG_LEN);
}

void lw_set_gprs_sbuf(uint8_t *msg)
{
	gprs_info.sbuf_info.sbuf_len = strlen(msg);
	strcpy(gprs_info.sbuf_info.sbuf, msg);
}

int32_t lw_is_sbuf_empty(void)
{
	return ((gprs_info.sbuf_info.sbuf_len > 0) ? 0:1);
}

static void lw_clean_gprs_rbuf(void)
{
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	memset(gprs_info.rbuf_info.rbuf, 0, sizeof(gprs_info.rbuf_info.rbuf)/sizeof(gprs_info.rbuf_info.rbuf[0]));
	gprs_info.rbuf_info.rbuf_index = 0;
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void lw_gprs_init(void)
{
	com_para_t com_para;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOC, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);//测试

	GPIO_InitStructure.GPIO_Pin = M_RST|M_ON;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	MG323_POWERSWITCH();
	MG323_NORM();


	//测试
	/*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_9);
	GPIO_SetBits(GPIOA, GPIO_Pin_10);	*/
	
	/* uart init */	
	com_para.baudrate = 115200;
	com_para.data_9bit = false;
	com_para.parity = 0;
	com_para.port = 1;//2;
	com_para.twoStopBits = false;
	com_init(&com_para);
	
}

void lw_start_gprs_mode(void)
{
	memset(&gprs_info, 0, sizeof(gprs_info));
	gprs_info.gprs_status = GPRS_STATUS_NOINIT;
	gprs_info.ate0 = 0;
	gprs_info.send_status.quit_socket = false;
	gprs_info.send_status.in_tp_mode = false;
	lw_clean_gprs_sbuf();
	lw_clean_gprs_rbuf();
	
#if 0
	MG323_POWERSWITCH();
	delay(150);	/* >= 1S */
	MG323_POWERNORM();

	MG323_RESET();
	delay(100);	/* >= 10ms */
	MG323_NORM();
#endif
}

//#define gprs_send_cmd(x, y) com_send_string(x, y)
#define gprs_send_cmd(x, y,z ) com_send_nchar(x, y, z)

static void lw_gprs_waitfor_start(void)
{ 
	if(gprs_info.rbuf_info.rbuf_index < 20)
		return;
	if (NULL != strstr(gprs_info.rbuf_info.rbuf, "SYSSTART"))
	{
		memset(&gprs_info, 0, sizeof(gprs_info));
		gprs_info.ate0 = 0;
		gprs_info.gprs_status = GRPS_STATUS_START;
	}

	lw_clean_gprs_rbuf();
	return ;
}

/* 关闭回显*/
static void lw_send_ate0(void)
{
	lw_clean_gprs_rbuf();
	gprs_info.gprs_status = GPRS_STATUS_ATE;
	gprs_send_cmd(1, ATE0, strlen(ATE0));
}

static void lw_send_get_imei(void)
{
	if(gprs_info.rbuf_info.rbuf[2]== 'O' && gprs_info.rbuf_info.rbuf[3] == 'K')
	{
		lw_clean_gprs_rbuf();
		gprs_info.gprs_status = GPRS_STATUS_GET_IMEI;
		//lw_gprs_get_imei();
		gprs_send_cmd(1, GET_IMEI, strlen(GET_IMEI));
	}
	else
	{
		lw_clean_gprs_rbuf();
		gprs_send_cmd(1, ATE0, strlen(ATE0));
	}
}

static void lw_send_nettype(void)
{
	if(!lw_gprs_get_imei())
	{
		lw_clean_gprs_rbuf();
		gprs_info.gprs_status = GPRS_STATUS_NETTYPE;
		gprs_send_cmd(1, SET_NETTYPE, strlen(SET_NETTYPE));
	}
	else
	{
		lw_clean_gprs_rbuf();
		gprs_send_cmd(1, GET_IMEI, strlen(GET_IMEI));
	}
}

static void lw_send_apntype(void)
{
	if(gprs_info.rbuf_info.rbuf[4]== 'O' && gprs_info.rbuf_info.rbuf[5] == 'K')
	{
		lw_clean_gprs_rbuf();
		gprs_info.gprs_status = GPRS_STATUS_APNTYPE;
		gprs_send_cmd(1, SET_NETTYPE, strlen(SET_NETTYPE));
	}
	else
	{
		lw_clean_gprs_rbuf();
		gprs_send_cmd(1, SET_NETTYPE, strlen(SET_NETTYPE));
	}
}

static void lw_send_srvtype(void)
{
	if(gprs_info.rbuf_info.rbuf[4]== 'O' && gprs_info.rbuf_info.rbuf[5] == 'K')
	{
		lw_clean_gprs_rbuf();
		gprs_info.gprs_status = GPRS_STATUS_SRVTYPE;
		gprs_send_cmd(1, SET_SRVTYPE, strlen(SET_SRVTYPE));
	}
	else
	{
		lw_clean_gprs_rbuf();
		gprs_send_cmd(1, SET_APNTYPE, strlen(SET_APNTYPE));
	}

}

static void lw_send_conid(void)
{
	if(gprs_info.rbuf_info.rbuf[4]== 'O' && gprs_info.rbuf_info.rbuf[5] == 'K')
	{
		lw_clean_gprs_rbuf();
		gprs_info.gprs_status = GPRS_STATUS_CONID;
		gprs_send_cmd(1, SET_CONID, strlen(SET_CONID));
	}
	else
	{
		lw_clean_gprs_rbuf();
		gprs_send_cmd(1, SET_SRVTYPE, strlen(SET_SRVTYPE));
	}
}

static void lw_send_srvurl(void)
{
	if(gprs_info.rbuf_info.rbuf[4]== 'O' && gprs_info.rbuf_info.rbuf[5] == 'K')
	{
		lw_clean_gprs_rbuf();
		gprs_info.gprs_status = GPRS_STATUS_SRVURL;
		gprs_send_cmd(1, SET_SRV_URL_1, strlen(SET_SRV_URL_1));
	}
	else
	{
		lw_clean_gprs_rbuf();
		gprs_send_cmd(1, SET_CONID, strlen(SET_CONID));
	}
}

static void lw_send_socket_open(void)
{
	if(gprs_info.rbuf_info.rbuf[4]== 'O' && gprs_info.rbuf_info.rbuf[5] == 'K')
	{
		lw_clean_gprs_rbuf();
		gprs_info.gprs_status = GPRS_STATUS_SOCKET_OPEN;
		gprs_send_cmd(1, OPEN_CON0, strlen(OPEN_CON0));
	}
	else
	{
		lw_clean_gprs_rbuf();
		gprs_send_cmd(1, SET_SRV_URL_1, strlen(SET_SRV_URL_1));
	}
}

uint32_t pow10(uint32_t i)
{
    uint32_t ret;

    ret = 1;
    while(i-->0)
        ret *= 10;
    return ret;
}

static void lw_clear_gprs_ack_sentence(uint8_t *start)
{
	uint16_t tmp;
	uint16_t start_addr;
	uint8_t *p;

	p = start;
	while(*p != 0x0A) 
		p++;
	start_addr = p - gprs_info.rbuf_info.rbuf;
	GPRS_RXNE_IRQ_DISABLE();
	for(tmp = start_addr + 1; tmp < gprs_info.rbuf_info.rbuf_index; tmp++)
		gprs_info.rbuf_info.rbuf[tmp - (start_addr + 1)] = gprs_info.rbuf_info.rbuf[tmp];
	gprs_info.rbuf_info.rbuf_index -= (start_addr + 1);
	memset(gprs_info.rbuf_info.rbuf + gprs_info.rbuf_info.rbuf_index, 0, 
		GPRS_MAX_MSG_LEN - gprs_info.rbuf_info.rbuf_index);
	GPRS_RXNE_IRQ_ENABLE();
}

static void lw_gprs_check_socket_opend(void)
{
	uint8_t i;
	uint8_t *p;
	uint8_t *ptmp;


	if (gprs_info.rbuf_info.rbuf_index < 4)	
		return ;
	if(gprs_info.rbuf_info.rbuf[4]== 'O' && gprs_info.rbuf_info.rbuf[5] == 'K')
	{
		if(gprs_info.rbuf_info.rbuf_index < 18) /* 没接收完毕*/
			return;
		if((p = memchr(gprs_info.rbuf_info.rbuf, ',', gprs_info.rbuf_info.rbuf_index)) == NULL)
		{
			gprs_info.send_status.session_max_len = 1024;
			return;
		}
		else if((p = memchr(p + 1, ',', 
			gprs_info.rbuf_info.rbuf_index - (p - gprs_info.rbuf_info.rbuf) - 1)) == NULL)
		{
			gprs_info.send_status.session_max_len = 1024;
			return;
		}
		ptmp = p +1;
		if((p = memchr(p +1, 0x0D, 
			gprs_info.rbuf_info.rbuf_index - (p - gprs_info.rbuf_info.rbuf) - 1)) == NULL)
		{
			return;	/* 没接收完毕 */
		}
		if(*(p+1) != 0x0A)
			return;

		*p = '\0';
		gprs_info.send_status.session_max_len = atoi(ptmp);

		gprs_info.send_status.socket_opend = true;
		//lw_clean_gprs_rbuf();
		lw_clear_gprs_ack_sentence(p);
	}
	else if(gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index - 1] == 0x0A) /* 接收错误完毕 */
	{
		//lw_clean_gprs_rbuf();
		lw_clear_gprs_ack_sentence(gprs_info.rbuf_info.rbuf + 4);
		gprs_send_cmd(1, OPEN_CON0, strlen(OPEN_CON0));
	}
}


static int32_t lw_gprs_check_socket_send_msg(void)
{
	static bool send_msg_ok = false;

	uint8_t tmp;
	uint8_t *p;

	if(false == send_msg_ok)
	{
	if((gprs_info.rbuf_info.rbuf_index < 6) ||
		(gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index - 1] != 0x0A))	 /* "\x00D\x00AOK\x00D\x00A" */
	{
		/* 没接受完毕 */
		return -1;
	}
	else if(gprs_info.rbuf_info.rbuf[2] != 'O')
	{
//com_send_message(2, gprs_info.rbuf_info.rbuf);
		/* 出错了 */
		//lw_clean_gprs_rbuf();
		lw_clear_gprs_ack_sentence(gprs_info.rbuf_info.rbuf + 2);
		send_msg_ok = false;
		return -2;
	}	
	else
	{
		/* 接收到OK以后，还有后面的主动上报信息^SISW: 0,1    或者   ^SISW: 0,1 */
		/*GPRS_RXNE_IRQ_DISABLE();
		for(tmp = 6; tmp < gprs_info.rbuf_info.rbuf_index; tmp++)
			gprs_info.rbuf_info.rbuf[tmp - 6] = gprs_info.rbuf_info.rbuf[tmp];
		gprs_info.rbuf_info.rbuf_index -= 6;
		GPRS_RXNE_IRQ_ENABLE();*/
		lw_clear_gprs_ack_sentence(gprs_info.rbuf_info.rbuf + 4);
		send_msg_ok = true;
	}
	}

	/* 处理自动返回的信息 : ^SISW: 0,1    或者   ^SISW: 0,1 */
	/*if(gprs_info.rbuf_info.rbuf_index < 14)*/
	if (gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index - 1] != 0x0A)
	{
		return -1;
	}
	p = memchr(gprs_info.rbuf_info.rbuf, ',', gprs_info.rbuf_info.rbuf_index);
	if(NULL == p)
	{
		lw_clean_gprs_rbuf();
		return -3; /*不可能*/
	}
	if(*(p +1) == '0')
	{
		/* 等待 */
		/*GPRS_RXNE_IRQ_DISABLE();
		for(tmp = 14; tmp < gprs_info.rbuf_info.rbuf_index; tmp++)
			gprs_info.rbuf_info.rbuf[tmp - 14] = gprs_info.rbuf_info.rbuf[tmp];
		gprs_info.rbuf_info.rbuf_index -= 14;
		GPRS_RXNE_IRQ_ENABLE();*/
		lw_clear_gprs_ack_sentence(gprs_info.rbuf_info.rbuf + 10);
		return -1;
	}

	/* 可以继续发送数据了 */
	send_msg_ok = false;
	//lw_clean_gprs_rbuf();
	lw_clear_gprs_ack_sentence(gprs_info.rbuf_info.rbuf + 4);
	return 0;

#if 0
	if(gprs_info.rbuf_info.rbuf[2]== 'O' && gprs_info.rbuf_info.rbuf[3] == 'K')	/* 发送到gprs成功 */
	{
		/* ^SISW: 0,1	如可以写入，GPRS模块主动回复*/
		if(memchr(gprs_info.rbuf_info.rbuf, 'W', GPRS_MAX_MSG_LEN) == NULL ||
			memchr(gprs_info.rbuf_info.rbuf, ',', GPRS_MAX_MSG_LEN) == NULL ||
			gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index - 1] != 0x0A) /* 未收到可写信息 ,　等待*/
		{
			com_send_message(2, "recv none SISW, wait..");
			return -1;
		}
		lw_clean_gprs_rbuf();
		return 0;
	}
	else
	{
		/*lw_clean_gprs_rbuf();
		gprs_send_cmd(1, gprs_info.tmpbuf2, gprs_info.tmpbuf2_len);*/

		/* 出错了 */
		lw_clean_gprs_rbuf();
		
		return -2;
	}
#endif
}

static void lw_send_socket_write(uint32_t msglen)
{
/*	if((gprs_info.rbuf_info.rbuf[4]== 'O' && gprs_info.rbuf_info.rbuf[5] == 'K') ||
		gprs_info.send_status.sec_msg )*/
	{
		/*gprs_info.rbuf_info.rbuf[GPRS_MAX_MSG_LEN] = '\0';
		com_send_message(2, gprs_info.rbuf_info.rbuf);
		if (strstr(gprs_info.rbuf_info.rbuf, "closed") != NULL){
			gprs_info.send_status.socket_closed_by_srv = true;
			return;
		}*/
	
		//lw_clean_gprs_rbuf();
		//gprs_info.send_status.socket_opend = true;
		gprs_info.gprs_status = GPRS_STATUS_SOCKET_WRITE;
		sprintf(gprs_info.tmpbuf, "AT^SISW=0,%d\x00D\x00A", msglen);
		//sprintf(gprs_info.tmpbuf, "AT^SISW=0,kueiurjiejfnejvimeuvnmenvmienvmkdnvkhvfkjvmkf vdiv nkdvmdvnmjdjnffnfjndvfjdvjfjvfjvdjfnvdjf d vfj djf jdvjfvjvfndjfnvjfjvfjvfjvjfvjfiuejvifuejveivrijfirjfrekjfksjdfkueifuniuceccccccccknvfjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjeuiyruiieunvem,ceincrintmbjnbjgnbhjnjmjrbngjkdfjgikfgjrhgrjhgjrfjgkfjgroooooooooooojjjjjjjjjjjjjjjjjjjjjjjjjklllllllllllirkjjfriufjirujtf847u584\x00D\x00A", msglen);		
		gprs_info.tmpbuf_len = strlen(gprs_info.tmpbuf);
		gprs_send_cmd(1, gprs_info.tmpbuf, strlen(gprs_info.tmpbuf));
	}
/*	else
	{
		lw_clean_gprs_rbuf();
		gprs_send_cmd(1, OPEN_CON0, strlen(OPEN_CON0));
	}*/	
}

static bool lw_gprs_is_socket_closed(void)
{
	/* ^SIS: 0,0,48,Remote Peer has closed the connection  服务器主动断开链接*/
	/*if(((ptemp = memchr(gprs_info.rbuf_info.rbuf, 'R', gprs_info.rbuf_info.rbuf_index)) != NULL) &&
		(*(ptemp+1) == 'e') &&
		((ptemp = memchr(ptemp+1, 'c',
			gprs_info.rbuf_info.rbuf_index - (ptemp - gprs_info.rbuf_info.rbuf) - 1)) != NULL) &&
		(*(ptemp+1) == 'l') ){*/
	gprs_info.rbuf_info.rbuf[GPRS_MAX_MSG_LEN-1] = '\0';
/*	com_send_message(2, "::");
	com_send_message(2, gprs_info.rbuf_info.rbuf);*/
	if (strstr(gprs_info.rbuf_info.rbuf, "closed") != NULL){
		//gprs_info.send_status.socket_closed_by_srv = true;
		return true;
	}

	return false;
}

static void lw_send_socket_write_msg(uint8_t *msg, uint32_t msg_len)
{
	#define END_FLAG "\x00D\x00A"
	//static uint8_t test= 0;
	uint8_t *p;
	uint8_t *ptmp;
	uint8_t *ptemp;
	uint8_t *pgprs_data_len;
	//static uint32_t un_ack_data_len = 0;


	
	/* ^SISW: 0,44,44 */
	if((p = memchr(gprs_info.rbuf_info.rbuf, ',', gprs_info.rbuf_info.rbuf_index)) == NULL)
	{
		return; /* 未接收完毕?? */
	}
	pgprs_data_len = p +1;
	if((p = memchr(p + 1, ',', 
		gprs_info.rbuf_info.rbuf_index - (p - gprs_info.rbuf_info.rbuf) - 1)) == NULL)
	{
		return; /* 未接收完毕?? */
	}
	ptmp = p +1;
	if((p = memchr(p +1, 0x0D, 
		gprs_info.rbuf_info.rbuf_index - (p - gprs_info.rbuf_info.rbuf) - 1)) == NULL)
	{
		return;	/* 没接收完毕 */
	}
	if(*(p+1) != 0x0A)
	{
		return;	/* 没接收完毕 */
	}
	
	/*
if(test++>=3)
{
		gprs_info.send_status.socket_closed_by_srv = true;
		return;
}*/		

	*(ptmp - 1) = '\0';
	if(0 == atoi(pgprs_data_len))
	{
		/* gprs返回值指示不能发送时，继续询问 */
		//lw_clean_gprs_rbuf();
		lw_clear_gprs_ack_sentence(pgprs_data_len);
		gprs_send_cmd(1, gprs_info.tmpbuf, gprs_info.tmpbuf_len);
		return;
	}

	*p = '\0';
	if(atoi(ptmp) >= 1680)
	{
		/* 认为断网 */
		com_send_message(2, "gprs socket lost");
		lw_clean_gprs_rbuf();
		return ;
	}
	

	//lw_clean_gprs_rbuf();
	lw_clear_gprs_ack_sentence(pgprs_data_len);
	gprs_info.gprs_status = GPRS_STATUS_SOCKET_WRITE_MSG;
	memcpy(gprs_info.tmpbuf2, msg, msg_len);
	sprintf(gprs_info.tmpbuf2 + msg_len, "%s", END_FLAG);
	gprs_info.tmpbuf2_len = msg_len + strlen(END_FLAG);
	gprs_send_cmd(1, gprs_info.tmpbuf2, gprs_info.tmpbuf2_len);
}

static void lw_send_socket_close(void)
{
/*	if(gprs_info.rbuf_info.rbuf[2]== 'O' && gprs_info.rbuf_info.rbuf[3] == 'K' &&
		gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index - 1] == 0x0A)*/
	if((true == gprs_info.send_status.socket_closed_by_srv) ||
		(gprs_info.rbuf_info.rbuf[2]== 'O' && gprs_info.rbuf_info.rbuf[3] == 'K' &&
		gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index - 1] == 0x0A))
		
	{
		lw_clean_gprs_rbuf();
		gprs_info.gprs_status = GPRS_STATUS_SOCKET_CLOSE;
		gprs_send_cmd(1, CLOSE_CON0, strlen(CLOSE_CON0));
	}
	else
	{
		/*lw_clean_gprs_rbuf();
		gprs_send_cmd(1, gprs_info.tmpbuf2, gprs_info.tmpbuf2_len);*/
	}
}

static void lw_socked_end(void)
{
	gprs_info.rbuf_info.rbuf[GPRS_MAX_MSG_LEN - 1] = '\0';
	if (strstr(gprs_info.rbuf_info.rbuf, "OK") != NULL) {
		gprs_info.gprs_status = /*GPRS_STATUS_SOCKET_OPEN;	*/GPRS_STATUS_NETTYPE;
		memset(&gprs_info.send_status, 0, sizeof(gprs_info.send_status));
		lw_clean_gprs_rbuf();
	}
	else {
		lw_clean_gprs_rbuf();
		gprs_send_cmd(1, CLOSE_CON0, strlen(CLOSE_CON0));
	}
}

static int32_t lw_gprs_get_imei(void)
{
	int32_t i;
	uint8_t tmp[16];
/*		
	if(gprs_info.rbuf_info.rbuf_index < 40) 
	{
		gprs_send_cmd(1, GET_IMEI);
		return -1;
	}*/

	/*com_send_message(1, "imei buf:"); 
	com_send_message(1, gprs_info.rbuf_info.rbuf);*/
		
	while(i++ < gprs_info.rbuf_info.rbuf_index)
	{
		if(gprs_info.rbuf_info.rbuf[i] == 0x0A)
		{
			if(i+15 >=  gprs_info.rbuf_info.rbuf_index)
			{
				//lw_clean_gprs_rbuf();					
				break;
			}
			else if(isdigit(gprs_info.rbuf_info.rbuf[i+1]) && isdigit(gprs_info.rbuf_info.rbuf[i+15]) )
			{
				memcpy(gprs_info.imei, &(gprs_info.rbuf_info.rbuf[i+1]), 15);
				com_send_message(1, "imei:");
				memcpy(tmp, gprs_info.imei, 15);
				tmp[15] = '\0';
				com_send_message(1, tmp);
				return 0;
			}
		}	
	}
	//lw_clean_gprs_rbuf();
	return -1;
}

static bool lw_reset_gprs(void)
{
	uint8_t *p;

	if (true == gprs_info.send_status.socket_closed_by_srv){
		gprs_info.send_status.socket_closed_by_srv = false;
		lw_clean_gprs_rbuf();
		MG323_RESET();
		delay(20);
		MG323_NORM();
	}
	
	gprs_info.rbuf_info.rbuf[GPRS_MAX_MSG_LEN - 1] = '\0';
	if ((p = strstr (gprs_info.rbuf_info.rbuf, "SYSSTART")) == NULL)
		return false;
	
	return true;
}

static void lw_check_gprs_stop(void)
{
	static int32_t pre_status = GPRS_STATUS_NOINIT;
	static int32_t g_status_count = 0;

	if (pre_status == gprs_info.gprs_status)
		g_status_count++;
	else {
		pre_status = gprs_info.gprs_status;
		g_status_count = 0;
	}
	if (g_status_count > 1000) {
		MG323_POWERNORM();
		delay(100);
		MG323_POWERSWITCH();
		delay(100);
		g_status_count = 0;
		gprs_info.gprs_status = GPRS_STATUS_NOINIT;
		memset(&gprs_info.send_status, 0, sizeof(gprs_info.send_status));
		lw_clean_gprs_rbuf();
		lw_clean_gprs_sbuf();		
	}
	
}

bool is_tp_ready(void)
{
	return gprs_info.send_status.in_tp_mode;
}

static int32_t lw_set_socket_tp_mode(void)
{
	Timer1=200;
	while (Timer1) ;
	lw_clean_gprs_rbuf();
	gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP;
	gprs_send_cmd(1, MG323_TP_TRANS, strlen(MG323_TP_TRANS));

	debug_printf_m("lw_set_socket_tp_mode");
	
	return 0;
}

static int32_t lw_send_socket_tp_mode(void)
{
	int32_t ret;

	ret = -1;
	
	if ((gprs_info.rbuf_info.rbuf_index < 6) || 
		(gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index - 1] != 0x0A))
		return -1;

	gprs_info.rbuf_info.rbuf[GPRS_MAX_MSG_LEN - 1] = '\0';
	if (strstr(gprs_info.rbuf_info.rbuf, "OK") != NULL) {	
		Timer1 = 200;
		while(Timer1) ;
		gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_FINISH;/*透传开始等待结束*/
		gprs_info.send_status.in_tp_mode = true;
		lw_clean_gprs_rbuf();
		debug_printf_m("start tp.");
		ret = 0;
	}
	else if (strstr(gprs_info.rbuf_info.rbuf, "ERR") != NULL) {
		lw_clean_gprs_rbuf();
		ret = -2;
	}
	else {
		lw_set_socket_tp_mode();
	}

	debug_printf_m("lw_send_socket_tp_mode");
	return ret;
}

static int32_t lw_set_socket_tp_mode_finish(void)
{
	if ((gprs_info.rbuf_info.rbuf_index < 6) || 
		(gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index - 1] != 0x0A))
		return -1;

	gprs_info.rbuf_info.rbuf[GPRS_MAX_MSG_LEN - 1] = '\0';
	if (strstr(gprs_info.rbuf_info.rbuf, "OK") != NULL) {	/*透传结束*/
		gprs_info.gprs_status = GPRS_STATUS_SOCKET_OPEN;
		gprs_info.send_status.in_tp_mode = false;
		lw_clean_gprs_rbuf();
		debug_printf_m("end tp.");
	}

	debug_printf_m("lw_set_socket_tp_mode_finish");
	return 0;
}

int32_t lw_gprs_tcp_create_send(void)
{
	int32_t  ret;
	//gprs_info.gprs_status = GPRS_STATUS_NOINIT;

/*	if(lw_is_sbuf_empty())
		return -1;*/
		
/*	if(gprs_info.ate0 == 0)
	{
		com_send_message(1, "\r\n");
		com_send_message(1, "rbuf_index:");
		com_send_hex(1, gprs_info.rbuf_info.rbuf_index);
		com_send_message(1, "\r\n");
		for(i=0;i<gprs_info.rbuf_info.rbuf_index;i++)
			com_send_hex(1, gprs_info.rbuf_info.rbuf[i]);
		com_send_message(1, "\r\n");
		
		if(gprs_info.rbuf_info.rbuf[2] == 'O' && gprs_info.rbuf_info.rbuf[3] == 'K')
		{
			lw_clean_gprs_rbuf();
			gprs_info.ate0 = 1;
		}
		else
		{
			lw_clean_gprs_rbuf();
			lw_send_ate0();
			return -1;
		}
	}*/

	/* 因为gprs模块隔1~1.5分钟就重启 */
	/*if (NULL != strstr(gprs_info.rbuf_info.rbuf, "SYSSTART"))
	{
		gprs_info.ate0 = 0;
		gprs_info.gprs_status = GPRS_STATUS_NOINIT;
		//lw_clean_gprs_rbuf();
		//return -2;
	}*/
	if(gprs_info.rbuf_info.rbuf[0] != 0 &&
		gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index - 1] !=  0x0A){
		return -1; /* 未接收完信息 */
	}
		
	if (true == lw_gprs_is_socket_closed()){
		gprs_info.send_status.socket_closed_by_srv = true;
		gprs_info.gprs_status = GPRS_STATUS_SOCKET_WILL_CLOSED;
		lw_clean_gprs_rbuf();
		//return -2;
	}

	/* FIXME:遇到过SISR */
	gprs_info.rbuf_info.rbuf[GPRS_MAX_MSG_LEN - 1] = '\0';
	if (strstr(gprs_info.rbuf_info.rbuf, "^SISR") == gprs_info.rbuf_info.rbuf + 2) {
		lw_clear_gprs_ack_sentence(gprs_info.rbuf_info.rbuf + 2);	/* 删除这个 */
		return -1;
	}

	lw_check_gprs_stop();
	
	switch(gprs_info.gprs_status)
	{
		case GPRS_STATUS_NOINIT:
		/*	lw_gprs_waitfor_start();
			break;
		case GRPS_STATUS_START:*/
			lw_send_ate0();
			break;
		case GPRS_STATUS_ATE:
			lw_send_get_imei();
			break;
		case GPRS_STATUS_GET_IMEI:
			lw_send_nettype();
			break;
		case GPRS_STATUS_NETTYPE:
			lw_send_apntype();
			break;
		case GPRS_STATUS_APNTYPE:
			lw_send_srvtype();
			break;			
		case GPRS_STATUS_SRVTYPE:
			lw_send_conid();
			break;
		case GPRS_STATUS_CONID:
			lw_send_srvurl();
			break;
		case GPRS_STATUS_SRVURL:
			lw_send_socket_open();
			break;
		case GPRS_STATUS_SOCKET_OPEN:
			if(true == gprs_info.send_status.quit_socket)	/* 退出 */
			{
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_WILL_CLOSED;
				break;
			}

			if(gprs_info.send_status.socket_opend == false)
			{
				lw_gprs_check_socket_opend();
				break;
			}
			
			if(gprs_info.send_status.socket_opend == true) {
				if (is_send_cam2pgrs()) { /*优先发cam*/
					lw_set_socket_tp_mode();
				}
				else if(gprs_info.sbuf_info.sbuf_len > 0)
					lw_send_socket_write(gprs_info.sbuf_info.sbuf_len);
			}
			break;
		case GPRS_STATUS_SOCKET_TP:
			lw_send_socket_tp_mode();
			break;
		case GPRS_STATUS_SOCKET_TP_FINISH:
			lw_set_socket_tp_mode_finish();
			break;
		case GPRS_STATUS_SOCKET_WRITE:
			lw_send_socket_write_msg(gprs_info.sbuf_info.sbuf, 
				gprs_info.sbuf_info.sbuf_len);
			lw_clean_gprs_sbuf();
			if(true == gprs_info.send_status.socket_closed_by_srv){
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_WILL_CLOSED;
			}
			break;
		case GPRS_STATUS_SOCKET_WRITE_MSG:
			ret = lw_gprs_check_socket_send_msg();
			if(ret == 0)
			{
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_OPEN;
			}
			else if(ret == -2)	/* 出错*/
			{
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_WILL_CLOSED;
			}
			break;
		case GPRS_STATUS_SOCKET_WILL_CLOSED:
			lw_send_socket_close();
			break;
		case GPRS_STATUS_SOCKET_CLOSE:
			/*lw_clean_gprs_sbuf();*/

			/* 以下代码使得GPRS设备重新复位 */
			/*if (true == lw_reset_gprs()){
				gprs_info.gprs_status = GPRS_STATUS_MAX;
				memset(&gprs_info.send_status, 0, sizeof(gprs_info.send_status));
				lw_clean_gprs_rbuf();
			}*/

			/* 以下代码使得GPRS设备重新open链接 */
			lw_socked_end();
			break;			
		default:
			break;
	}

	return 0;
}

void lw_process_gprs_rbuf(uint8_t val)
{
	if(gprs_info.rbuf_info.rbuf_index >= GPRS_MAX_MSG_LEN)
		gprs_info.rbuf_info.rbuf_index = 0;
	gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index++] = val;
}

void lw_gprs_form_C_cmd(double lat, double lon, double speed, 
	double track/*航向*/, bool isvalid /*数据状态: 1:有效，0:无效*/)
{
	//#define C_STYPE "C:%f,%f,%f,%f,%d#"
	//#define C_STYPE "C:%f,%f,%f,%f,%d#"
	//#define C_STYPE "s:%f,%f,%f,%f,%d#"
	#define C_STYPE "s:%f,%f,%f,%f,%d,12345678#"
	//#define C_STYPE "s:%f,%f,1,2,%d,12345678#"
	//#define C_STYPE "s:2310.0216,11315.6558,000,096,1,12345678#"

	sprintf(gprs_info.sbuf_info.gps_sbuf, C_STYPE, lat, lon, speed, track, isvalid);
	gprs_info.sbuf_info.gps_sbuf_len = sizeof(gprs_info.sbuf_info.gps_sbuf);

	/*com_send_message(1, "C cmd:");
	com_send_message(1, gprs_info.sbuf_info.gps_sbuf);*/
}

static int32_t lw_set_sbuf_imei(void)
{

	//#define TEST_CMD_A "A:123456789#"
	#define TEST_CMD_A "D:123456789#"
	uint8_t A_tmp[50];
	uint8_t tmp[50];
	
	//if(lw_is_sbuf_empty())
	{
		//lw_set_gprs_sbuf(TEST_CMD_D);
		//lw_gprs_get_imei();
		if((gprs_info.imei[0])>'0' && (gprs_info.imei[1]>'0'))
		{		
			memcpy(A_tmp, gprs_info.imei, 15);
			A_tmp[15] = '\0';
			//sprintf(tmp, "A:%s#", A_tmp);	
			sprintf(tmp, "D:%s#", A_tmp);	
			lw_set_gprs_sbuf(tmp);
			//gprs_info.gprs_status = GPRS_STATUS_NETTYPE;
			return 0;
		}
	}

	return -1;
}

int32_t lw_gprs_send_gps_test(void)
{
	if(gprs_info.gprs_status == GPRS_STATUS_APNTYPE)
	{
		/* 到这里，已经获得imei */
		if(lw_set_sbuf_imei() != 0 )
			return -1;
	}
	else if(gprs_info.gprs_status == GPRS_STATUS_SOCKET_WRITE)
	{
		
	}
	else if(gprs_info.gprs_status == GPRS_STATUS_SOCKET_CLOSE)
	{
		//gprs_info.gprs_status =GPRS_STATUS_SOCKET_WRITE; /* 可以重复发送 */
	}
	else if(gprs_info.gprs_status == GPRS_STATUS_MAX)
	{
			gprs_info.gprs_status = GPRS_STATUS_NOINIT;
	}

	if(lw_is_sbuf_empty())
		if(gprs_info.sbuf_info.gps_sbuf_len > 0)
		{
			lw_set_gprs_sbuf(gprs_info.sbuf_info.gps_sbuf);
			
			//return 0;
		}
		
	lw_gprs_tcp_create_send();			
	
	return 0;
}

void test_gprs_1(void)
{
	#define TEST_CMD_D "D:123456789#"
	uint8_t D_tmp[25];
	uint8_t tmp[25];

	if(lw_is_sbuf_empty())
	{
		//lw_set_gprs_sbuf(TEST_CMD_D);
		//lw_gprs_get_imei();
		if((gprs_info.imei[0])>'0' && (gprs_info.imei[1]>'0'))
		{		
			memcpy(D_tmp, gprs_info.imei, 15);
			D_tmp[15] = '\0';
			sprintf(tmp, "D:%s#", D_tmp);	
			//sprintf(tmp, "A:%s#", D_tmp);	
			lw_set_gprs_sbuf(tmp);
			//gprs_info.gprs_status = GPRS_STATUS_NETTYPE;
		}
	}
	//else
	{
		lw_gprs_tcp_create_send();
	}
}


int32_t lw_gprs_send_cam_frame(void)
{
	//lw_send_socket_write();
	lw_cam_start_frame();

	while(lw_cam_stop_frame() != 0 ) ;
	lw_send_socket_write_msg(NULL, 0);
	
}

bool lw_gprs_isidle(void)
{
	bool isidle;

	if(gprs_info.send_status.socket_opend != true)	/* gprs not opened */
		isidle = false;
	else
		isidle = (lw_is_sbuf_empty() == 1) ? true: false;
	return isidle;
}


#define CAM_CMD_L "L:%X#\x00D\x00A"
void lw_send_cam_cmd_L(uint32_t frame_len)
{
	uint8_t tmp[30];
	
	lw_clean_gprs_rbuf();

	/* 长度 */
	sprintf(tmp, CAM_CMD_L, frame_len);
	lw_send_socket_write(strlen(tmp));
	
	/* 内容 */
	gprs_send_cmd(1, tmp, strlen(tmp));	
}

void lw_grps_send_cam_frame_len(uint32_t frame_len)
{
	/* 长度 */
	lw_send_socket_write(frame_len);

	/* 内容由DMA */
}

uint8_t testgprs[512];
int32_t lw_cam_direct_to_gprs(void)
{
	extern volatile unsigned int Timer1, Timer2;

	#define TEST_CMD_A "D:%s#"
	#define TP_END_FLAG "+++"

	bool finish;
	uint8_t buf[20];
	uint8_t imei[20];
	uint32_t len;
	uint8_t *p;
	
for(len=0;len<512;len++)	
testgprs[len] = len%10+'0';

	if (is_tp_ready()) {
		if(lw_cam_get_frame())
			return -1;

		/*传imei*/
		memcpy(imei, gprs_info.imei, 15);
		imei[15] = '\0';
		sprintf(buf,TEST_CMD_A, imei);
		gprs_send_cmd(1, buf, strlen(buf));

		/*传jpg长度*/
		sprintf(buf, CAM_CMD_L, lw_get_frame_len() + 5*2); /*5*2:头尾各5bytes*/
		//sprintf(buf, CAM_CMD_L, 8); /*5*2:头尾各5bytes*/
		gprs_send_cmd(1, buf, strlen(buf));

		#if 0
		gprs_send_cmd(1, "abcdefgh", 8);
		#elif 1
		for(len=0; len<30;len++) {
			gprs_send_cmd(1, testgprs, 512);
			Timer1 = 100;
			while(Timer1) ;
		}
		#else
		lw_cam_start_frame_();
		do {
			finish = lw_get_cam_data_to_gprs(&p, &len);
			if (len) {
				gprs_send_cmd(1, p, len);
			}
		} while (false == finish); 
		#endif
		
		Timer1 = 100;
		while(Timer1) ;
		gprs_send_cmd(1, TP_END_FLAG, strlen(TP_END_FLAG));
		Timer1 = 100;
		//while(Timer1) ;
		while(1) ;
	}

	return 0;
}
