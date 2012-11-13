#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "stm32f10x.h" //"stm32f10x_nvic.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "lw_gprs.h"
#include "log.h"
#include "lw_stm32_uart.h"
#include "ctrl_gps_cam.h"
#include "projects_conf.h"

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
#define SET_SRV_URL_1 "AT^SISS=0,address,\"socktcp://113.105.139.109:6969\"\x00D\x00A"
//#define SET_SRV_URL_1 "AT^SISS=0,address,\"socktcp://203.88.202.116:7788\"\x00D\x00A"
#define OPEN_CON0 "AT^SISO=0\x00D\x00A"
#define CLOSE_CON0 "AT^SISC=0\x00D\x00A"
#define GET_IMEI "AT+CGSN\x00D\x00A"
#define MG323_TP_TRANS "AT^IPENTRANS=0\x00D\x00A"

#define GPRS_RXNE_IRQ_ENABLE() USART_ITConfig(USART1, USART_IT_RXNE, ENABLE)
#define GPRS_RXNE_IRQ_DISABLE() USART_ITConfig(USART1, USART_IT_RXNE, DISABLE)

extern volatile unsigned int Timer1, Timer2;

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
	GPRS_STATUS_SOCKET_OPEN_SUCC,
	GPRS_STATUS_SOCKET_TP,
	GPRS_STATUS_SOCKET_TP_SUCC,
	GPRS_STATUS_SOCKET_TP_SEND_CAM,
	GPRS_STATUS_SOCKET_TP_SEND_GPS,
	GPRS_STATUS_SOCKET_TP_SEND_MSG,
	GPRS_STATUS_SOCKET_TP_FINISH,
	GPRS_STATUS_SOCKET_WRITE,
	GPRS_STATUS_SOCKET_WRITE_MSG,
	GPRS_STATUS_SOCKET_WILL_CLOSED,
	GPRS_STATUS_SOCKET_READ,
	GPRS_STATUS_SOCKET_CLOSE,
	GPRS_STATUS_SOCKET_CLOSE_OK,

	GPRS_STATUS_MAX,
}GPRS_MODULE_STATUS;

struct _gprs_info{
	uint8_t imei[15];
	bool send_imei_now;
	GPRS_MODULE_STATUS gprs_status;
	struct {
		uint32_t session_max_len;
	}send_status;
	struct {
		uint16_t rbuf_index;
		uint8_t rbuf[GPRS_MAX_MSG_LEN];
	}rbuf_info;
	struct {
		uint8_t sbuf_len;
		uint8_t sbuf[GPRS_MAX_MSG_LEN];
		uint8_t gps_sbuf_len;
		uint8_t gps_sbuf[GPS_MAX_MSG_LEN];
	}sbuf_info;
};
struct _gprs_info gprs_info;

#define SEQED_MSGS_MAX_LEN 64
#define SEQED_MSGS_MAX_NUM 10
struct _seqed_msgs {
	uint32_t gprs_rbuf_point;
	uint8_t msgs_num;
	uint8_t msgs[SEQED_MSGS_MAX_NUM][SEQED_MSGS_MAX_LEN];
	uint32_t msgtmp_len;
	uint8_t msgtmp[SEQED_MSGS_MAX_LEN];
};
struct _seqed_msgs seqed_msgs;
void lw_seqed_msgs_init(void)
{
	uint32_t i;

	seqed_msgs.msgtmp_len = 0;
	seqed_msgs.msgs_num = 0;
	seqed_msgs.gprs_rbuf_point = 0;
	for (i=0; i<SEQED_MSGS_MAX_NUM; i++)
		memset(seqed_msgs.msgs, '\0', SEQED_MSGS_MAX_LEN);
}

void lw_clean_seqed_msgs(void)
{
	uint32_t i;

	seqed_msgs.msgs_num = 0;
	for (i=0; i<SEQED_MSGS_MAX_NUM; i++)
		memset(seqed_msgs.msgs[i], '\0', SEQED_MSGS_MAX_LEN);
}
static void get_seqed_msgs(uint32_t start, uint32_t end)
{
	static bool newline = false;
	uint32_t i;
	uint8_t *p;

	p = gprs_info.rbuf_info.rbuf;
	p += start;
	for (i=0; i<end-start; i++) {
		if ((p[i] == 0x0D) || (p[i] == 0x0A)) {
			
			if (!newline) {
				if (seqed_msgs.msgtmp_len) {
					memcpy(seqed_msgs.msgs[seqed_msgs.msgs_num++], 
						seqed_msgs.msgtmp, seqed_msgs.msgtmp_len);
				}
				if (seqed_msgs.msgs_num >= SEQED_MSGS_MAX_NUM)
					seqed_msgs.msgs_num = 0;
			}
			seqed_msgs.msgtmp_len = 0;
			newline = true;
		}
		else {
			newline = false;	
			seqed_msgs.msgtmp[seqed_msgs.msgtmp_len++] = p[i];
		}
	}
}

void lw_form_seqed_msgs(void)
{
	uint16_t i;
	uint32_t gindex;
	uint32_t spoint;
	
	gindex = gprs_info.rbuf_info.rbuf_index;
	spoint = seqed_msgs.gprs_rbuf_point;
	if (gindex == spoint)
		return ;
	if (gindex > spoint) {
		get_seqed_msgs(spoint, gindex);
	}
	else {
		get_seqed_msgs(spoint, GPRS_MAX_MSG_LEN - 1);
		get_seqed_msgs(0, gindex);
	}

	seqed_msgs.gprs_rbuf_point = gindex;

	for(i=0;i<seqed_msgs.msgs_num;i++) {
		log_write_ack(seqed_msgs.msgs[i], strlen(seqed_msgs.msgs[i]));
		//log_write_ack("\x00A", 1);
	}
}

static bool lw_get_seqed_msg(uint8_t *p)
{
	uint32_t i;
	
	for(i=0;i<seqed_msgs.msgs_num;i++) {
		if (strstr(seqed_msgs.msgs[i], p))
			return true;
	}

	return false;
}


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
	uint32_t i;
	
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

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
	for (i=0;i<65536;i++)
		;
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
}

void lw_start_gprs_mode(void)
{
	memset(&gprs_info, 0, sizeof(gprs_info));
	gprs_info.send_imei_now = true;
	gprs_info.gprs_status = GPRS_STATUS_NOINIT;
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
#define gprs_send_cmd(x, y,z ) {com_send_nchar(x, y, z);log_write_snd(y,z);}


static int32_t lw_set_sbuf_imei(void)
{
	uint8_t A_tmp[20];
	uint8_t tmp[20];
	
	//if(lw_is_sbuf_empty())
	{
		//lw_set_gprs_sbuf(TEST_CMD_D);
		//lw_gprs_get_imei();
		if((gprs_info.imei[0])>'0' && (gprs_info.imei[1]>'0'))
		{		
			memcpy(A_tmp, gprs_info.imei, 15);
			A_tmp[15] = '\0';
			sprintf(tmp, "A:%s#", A_tmp);	
			//sprintf(tmp, "D:%s#", A_tmp);	
			lw_set_gprs_sbuf(tmp);
			//gprs_info.gprs_status = GPRS_STATUS_NETTYPE;
			gprs_info.send_imei_now = true;
			return 0;
		}
	}

	return -1;
}


void lw_get_gps_to_sbuf(void)
{
	if(lw_is_sbuf_empty())
		if(gprs_info.sbuf_info.gps_sbuf_len > 0)
			lw_set_gprs_sbuf(gprs_info.sbuf_info.gps_sbuf);
}




static void lw_send_ate0(void)
{
	extern volatile unsigned int Timer1, Timer2;
	
	gprs_info.gprs_status = GPRS_STATUS_ATE;
	gprs_send_cmd(1, ATE0, strlen(ATE0));
}

static void lw_send_get_imei(void)
{
	extern volatile unsigned int Timer1, Timer2;
	
	if(lw_get_seqed_msg("OK") /*返回OK*/)
	{
		gprs_info.gprs_status = GPRS_STATUS_GET_IMEI;
		//lw_gprs_get_imei();
		gprs_send_cmd(1, GET_IMEI, strlen(GET_IMEI));
	}
	else
	{
		//gprs_send_cmd(1, ATE0, strlen(ATE0));
	}
}

static bool lw_gprs_get_imei(void)
{
	uint32_t i;
	uint8_t tmp[20];
	
	for(i=0;i<seqed_msgs.msgs_num;i++) {
		//if (isdigit(seqed_msgs.msgs[i][2]) && isdigit(seqed_msgs.msgs[i][10]) ) {
		if ((seqed_msgs.msgs[i][2] > 0) && (seqed_msgs.msgs[i][10]) > 0) {
			memcpy(gprs_info.imei, seqed_msgs.msgs[i], 15);
			com_send_message(1, "imei:");
			memcpy(tmp, gprs_info.imei, 15);
			tmp[15] = '\0';
			com_send_message(1, tmp);

			return true;
		}
	}

	return false;
}

static void lw_send_nettype(void)
{
	extern volatile unsigned int Timer1, Timer2;

	if (lw_get_seqed_msg("OK")) {
		if(lw_gprs_get_imei())
		{
			gprs_info.gprs_status = GPRS_STATUS_NETTYPE;
			gprs_send_cmd(1, SET_NETTYPE, strlen(SET_NETTYPE));
		}
		else 
			gprs_send_cmd(1, GET_IMEI, strlen(GET_IMEI));
	}
	else
	{
		//gprs_send_cmd(1, GET_IMEI, strlen(GET_IMEI));
	}
}

static void lw_send_apntype(void)
{
	extern volatile unsigned int Timer1, Timer2;
	
	if(lw_get_seqed_msg("OK")/*收到OK*/)
	{
		gprs_info.gprs_status = GPRS_STATUS_APNTYPE;
		gprs_send_cmd(1, SET_APNTYPE, strlen(SET_APNTYPE));
	}
	else if (lw_get_seqed_msg("ERROR"))
	{
		gprs_send_cmd(1, SET_NETTYPE, strlen(SET_NETTYPE));
	}
	else {

	}
}

static void lw_send_srvtype(void)
{
	extern volatile unsigned int Timer1, Timer2;
	
	if(lw_get_seqed_msg("OK") /*收到OK*/)
	{
		gprs_info.gprs_status = GPRS_STATUS_SRVTYPE;
		gprs_send_cmd(1, SET_SRVTYPE, strlen(SET_SRVTYPE));
	}
	else if (lw_get_seqed_msg("ERROR"))
	{
		gprs_send_cmd(1, SET_APNTYPE, strlen(SET_APNTYPE));
	}
	else {

	}
}

static void lw_send_conid(void)
{
	extern volatile unsigned int Timer1, Timer2;
	
	if(lw_get_seqed_msg("OK") /*收到OK*/)
	{
		gprs_info.gprs_status = GPRS_STATUS_CONID;
		gprs_send_cmd(1, SET_CONID, strlen(SET_CONID));
	}
	else if (lw_get_seqed_msg("ERROR"))
	{
		gprs_send_cmd(1, SET_SRVTYPE, strlen(SET_SRVTYPE));
	}
	else {

	}
}

static void lw_send_srvurl(void)
{
	extern volatile unsigned int Timer1, Timer2;
	
	if(lw_get_seqed_msg("OK") /*收到OK*/)
	{
		gprs_info.gprs_status = GPRS_STATUS_SRVURL;
		gprs_send_cmd(1, SET_SRV_URL_1, strlen(SET_SRV_URL_1));
	}
	else if (lw_get_seqed_msg("ERROR"))
	{
		gprs_send_cmd(1, SET_CONID, strlen(SET_CONID));
	}
	else {

	}
}

static void lw_send_socket_open(void)
{
	extern volatile unsigned int Timer1, Timer2;
	
	if(lw_get_seqed_msg("OK") /*收到OK*/)
	{
		gprs_info.gprs_status = GPRS_STATUS_SOCKET_OPEN;
		gprs_send_cmd(1, OPEN_CON0, strlen(OPEN_CON0));
	}
	else if (lw_get_seqed_msg("ERROR"))
	{
		gprs_send_cmd(1, SET_SRV_URL_1, strlen(SET_SRV_URL_1));
	}
	else {

	}
}

static void lw_gprs_check_socket_opend_succ(void)
{
	static bool sk_opend_ok = false;
	uint32_t i;
	uint8_t *p;
	uint8_t *ptmp;

	if (!sk_opend_ok) {
		if (lw_get_seqed_msg("OK") /*接收到OK*/) {
			sk_opend_ok = true;
		}
		else if (lw_get_seqed_msg("ERROR") /*接收到ERROR*/) {
			gprs_send_cmd(1, OPEN_CON0, strlen(OPEN_CON0));
		}
		else {/*等待*/

		}
	}
	if (sk_opend_ok){
		for(i=0;i<seqed_msgs.msgs_num;i++) {
			if ((p = strstr(seqed_msgs.msgs[i], "SISW")) &&
					(ptmp = strstr(p+1, ",")) &&
					(p = strstr(ptmp+1, ","))) { /*收到类似"^SISW: 0,1,1024" 的主动上报信息*/
				*(p) = '\0';
				if (atoi(ptmp+1)) {
					sk_opend_ok = false; /*为了下次重新打开可再使用*/
					gprs_info.send_status.session_max_len = atoi(p+1);
					gprs_info.gprs_status = GPRS_STATUS_SOCKET_OPEN_SUCC;
					break;
				}
			}
		}
	}
}


static int32_t lw_send_socket_tp_mode(void)
{
	gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP;
	gprs_send_cmd(1, MG323_TP_TRANS, strlen(MG323_TP_TRANS));

	return 0;
}

static int32_t lw_gprs_check_socket_tp_mode_succ(void) 
{
	if (lw_get_seqed_msg("OK") /*收到OK*/) {
		gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_SUCC;
	}
	else if (lw_get_seqed_msg("ERROR") /*收到ERROR*/) {
		gprs_send_cmd(1, MG323_TP_TRANS, strlen(MG323_TP_TRANS));
	}
	else { /*等待*/

	}

	return 0;
}

#define CAM_CMD_L "L:%X#"

#include "lw_vc0706.h"

static bool wait_l_cmd;
static void get_cam_strem_pre_init(void)
{
	wait_l_cmd = false;
}

#if defined(STM_CAR)
/*
	0:finish
	-1:err
	1:continue
*/
static int32_t get_cam_stream(void)
{
	extern volatile unsigned int wait_l_cmd_timeout;
	bool finish;
	uint8_t cnt;
	uint8_t *p;
	uint32_t len;
	
	if (!wait_l_cmd) {
		cnt = 0;
		do {
			finish = lw_get_cam_data_to_gprs(&p, &len);
			if (!len && !finish) {
				Timer1 = 100;
				while(Timer1) ;
				if (cnt++ >= 10) {
					goto err;
				}
			}
		} while(!len && !finish);
		if (len) {
			wait_l_cmd_timeout = 3000;
			gprs_send_cmd(1, p, len);
		}
		wait_l_cmd = true;
		if (true == finish) {
			goto fs;
		} 
		
	}
	else {
		if (lw_get_seqed_msg("L:"))
			wait_l_cmd = false;
		else if (wait_l_cmd_timeout == 0) {
			goto err;
		}
	}

	return 1;

fs:
	wait_l_cmd = false;
	lw_cam_stop_frame_();
	return 0;
err:
	wait_l_cmd = false;
	lw_cam_stop_frame_();
	return -1;
}
#endif

/*
	-1:err
	0:finish
	1:continue
*/
	#define TP_GET_FRAME	0
	#define TP_SEND_PREMSG	1
	#define TP_SEND_MSG 	2
	#define TP_SEND_END 	3
	#define TP_FINISH	 	4
	static uint8_t send_cam_step; 
static void send_cam_stream_param_init(void)
{
	send_cam_step = TP_GET_FRAME;
}

static int32_t lw_gprs_send_cam(void)
{
#if defined(STM_CAR)
	extern volatile unsigned int Timer1, Timer2;

	#define TEST_CMD_A "A:%s#"
	#define TP_END_FLAG "+++"

	int32_t ret;
	uint8_t buf[20];
	uint8_t imei[20];

	#if 0
	static uint8_t cnt = 0;
	static uint8_t testtp[512];
	#endif

	switch (send_cam_step) {
	case TP_GET_FRAME:
		if(!lw_cam_get_frame()) { /*ok*/
			send_cam_step = TP_SEND_PREMSG;
			ret = 1;
		}
		else 
			ret = -1;
		
		break;
	case TP_SEND_PREMSG:
		/*传imei*/
		memcpy(imei, gprs_info.imei, 15);
		imei[15] = '\0';
		sprintf(buf,TEST_CMD_A, imei);
		gprs_send_cmd(1, buf, strlen(buf));

		/*传jpg长度*/
		sprintf(buf, CAM_CMD_L, lw_get_frame_len() + 5*2); /*5*2:头尾各5bytes*/
		//sprintf(buf, CAM_CMD_L, 8); /*5*2:头尾各5bytes*/
		gprs_send_cmd(1, buf, strlen(buf));

		/*使能jpg帧数据*/
		lw_cam_start_frame_();
		
		send_cam_step = TP_SEND_MSG;
		get_cam_strem_pre_init();
		ret = 1;
		break;
	case TP_SEND_MSG:
		/*传jpg数据*/
		#if 0
		gprs_send_cmd(1, "abcdefgh", 8);
		step = TP_SEND_END;
		#elif 0
		for(len=0;len<512;len++)	
			testtp[len] = len%10+'0';
		//for(len=0; len<30;len++) {
			gprs_send_cmd(1, testtp, 512);
			Timer1 = 100;
			while(Timer1) ;
		//}
		if (cnt++ >= 30) {
			step = TP_SEND_END;
			cnt = 0;
		}
		#else

		ret = get_cam_stream();
		if (ret == 0) {
			send_cam_step = TP_FINISH;
			ret = 1;
		}
		#endif

		break;

	case TP_FINISH:
		send_cam_step = TP_GET_FRAME;
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
	
#else
	return 0;
#endif
}

static bool lw_gprs_sending_gps(uint8_t *buf, uint32_t buflen) 
{
	gprs_send_cmd(1, buf, buflen);

	return true;
}

static bool lw_gprs_close(void)
{
	gprs_send_cmd(1, CLOSE_CON0, strlen(CLOSE_CON0));

	return true;
}

static void lw_check_gprs_close(void)
{	
	if(lw_get_seqed_msg("OK") /*返回OK*/)
	{
		gprs_info.gprs_status = GPRS_STATUS_NOINIT;
	}
	else if (lw_get_seqed_msg("ERROR")){
		gprs_send_cmd(1, CLOSE_CON0, strlen(CLOSE_CON0));
	}
	else {
		//gprs_send_cmd(1, CLOSE_CON0, strlen(CLOSE_CON0));
	}
}

static void lw_gprs_send_tp_end(void) 
{
	#define TP_END_FLAG "+++"

	Timer1 = 100;
	while(Timer1) ;
	gprs_send_cmd(1, TP_END_FLAG, strlen(TP_END_FLAG));
	Timer1 = 100;
	while(Timer1) ;
}

int32_t lw_gprs_tcp_send_data(void)
{
	int32_t ret;
	
	debug_printf_m("start lw_gprs_tcp_send_data");	
	if (lw_get_seqed_msg("START")) {
		gprs_info.gprs_status = GPRS_STATUS_NOINIT;
	}
	//else if (lw_get_seqed_msg("closed") || (gprs_info.gprs_status ==GPRS_STATUS_SOCKET_OPEN))
	else if (lw_get_seqed_msg("closed")) {
		gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_FINISH;
	}

	debug_printf_s("gprs_status=");
	debug_printf_h(gprs_info.gprs_status);
	debug_printf_m("");
	switch(gprs_info.gprs_status)
	{
		case GPRS_STATUS_NOINIT:
			if (!is_send_cam() && !is_send_gps() && !(gprs_info.send_imei_now))
				break;
			lw_start_gprs_mode();
			lw_seqed_msgs_init();
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
			if(lw_set_sbuf_imei() != 0 )
				return -1;
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
			lw_gprs_check_socket_opend_succ();
			break;
		case GPRS_STATUS_SOCKET_OPEN_SUCC:
			lw_send_socket_tp_mode();
			break;
		case GPRS_STATUS_SOCKET_TP:
			lw_gprs_check_socket_tp_mode_succ();
			break;
		case GPRS_STATUS_SOCKET_TP_SUCC:
			if (true == gprs_info.send_imei_now) {
				gprs_info.send_imei_now = false;
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_SEND_GPS;
			}
			#if defined(STM_CAR)
			else if (is_send_cam()) {
				set_no_send_cam();
				send_cam_stream_param_init();
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_SEND_CAM;
			}
			#endif
			else if (is_send_gps() && (gprs_info.sbuf_info.sbuf_len > 0)) {
				set_no_send_gps();
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_SEND_GPS;
			}
			else { /*没得发*/
				//gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_FINISH;
			}

			break;
		case GPRS_STATUS_SOCKET_TP_SEND_CAM:
			ret = lw_gprs_send_cam();
			if (ret == 0){
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_SUCC;
			}
			else if (ret == -1) {
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_FINISH;
			}

			break;
		case GPRS_STATUS_SOCKET_TP_SEND_GPS:
			if (lw_gprs_sending_gps(gprs_info.sbuf_info.sbuf, gprs_info.sbuf_info.sbuf_len)) {
				gprs_info.gprs_status = GPRS_STATUS_SOCKET_TP_SUCC;
				lw_clean_gprs_sbuf();
			}
			break;
		case GPRS_STATUS_SOCKET_TP_FINISH:
			lw_gprs_send_tp_end();
			gprs_info.gprs_status = GPRS_STATUS_SOCKET_CLOSE;
			break;
		case GPRS_STATUS_SOCKET_CLOSE:
			lw_gprs_close();
			gprs_info.gprs_status = GPRS_STATUS_SOCKET_CLOSE_OK;
			break;			
		case GPRS_STATUS_SOCKET_CLOSE_OK:
			lw_check_gprs_close();
			break;
		default:
			break;
	}

	return 0;
}

void lw_process_gprs_rbuf(uint8_t val)
{
	gprs_info.rbuf_info.rbuf[gprs_info.rbuf_info.rbuf_index++] = val;
	if(gprs_info.rbuf_info.rbuf_index >= GPRS_MAX_MSG_LEN)
		gprs_info.rbuf_info.rbuf_index = 0;
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
