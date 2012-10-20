#include <stdio.h>
#include <string.h>
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "lw_vc0706.h"
#include "lw_stm32_uart.h"
#include "lw_stm32_dma.h"

#define LW_VC0706_FLAG_RECV 0x56
#define LW_VC0706_FLAG_ACK 0x76


#define LW_VC0706_CMD_SYSTEM_RESET 0x26
#define LW_VC0706_CMD_READ_FBUF 0x32
#define LW_VC0706_CMD_GET_FBUF_LEN 0x34
#define LW_VC0706_CMD_FBUF_CTRL 0x36

#define LW_VC0706_FBUF_CURRENT_FRAME 0x00
#define LW_VC0706_FBUF_NEXT_FRAME 0x01


#define VC0706_MAX_CMD_LEN 40

#define VC0706_RXNE_IRQ_ENABLE() USART_ITConfig(USART3, USART_IT_RXNE, ENABLE)
#define VC0706_RXNE_IRQ_DISABLE() USART_ITConfig(USART3, USART_IT_RXNE, DISABLE)

enum vc0706_status {
	VC0706_STATUS_NOINIT = 0,
	VC0706_STATUS_SYSTEM_RESET,
	VC0706_STATUS_STOP_CFBUF,
	VC0706_STATUS_GET_LEN,
	VC0706_STATUS_READ_FBUF,
	VC0706_STATUS_TRANS_FINISHED,
};

struct vc0706_info_{
	enum vc0706_status status;
	struct {
		uint8_t rbuf[VC0706_MAX_CMD_LEN];
		uint8_t rbuf_len;
		uint32_t frame_len;
	}rbuf_info;
	struct {
		uint8_t sbuf[VC0706_MAX_CMD_LEN];
		uint8_t sbuf_len;
	}sbuf_info;
};
struct vc0706_info_ vc0706_info;

uint8_t lw_vc0706_get_rbuf(uint8_t *buf)
{
	memcpy(buf, vc0706_info.rbuf_info.rbuf, vc0706_info.rbuf_info.rbuf_len);
	return vc0706_info.rbuf_info.rbuf_len;
}

void lw_vc0706_recv(uint8_t val)
{
	vc0706_info.rbuf_info.rbuf[vc0706_info.rbuf_info.rbuf_len++] = val;
}

void lw_vc0706_clean_rbuf(void)
{
	vc0706_info.rbuf_info.rbuf_len = 0;
	memset(vc0706_info.rbuf_info.rbuf, 0, VC0706_MAX_CMD_LEN);
}

void lw_vc0706_init(void)
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

void lw_vc0706_param_init(void)
{
	memset(&vc0706_info, 0, sizeof(vc0706_info));
	vc0706_info.status = VC0706_STATUS_NOINIT;
}

static uint32_t lw_vc0706_fill_command(uint8_t *buf, uint8_t vc0706_flag,
	uint8_t sequence, uint8_t vc0706_cmd, uint8_t data_len, uint8_t *data)
{
	buf[0] = vc0706_flag;
	buf[1] = sequence;
	buf[2] = vc0706_cmd;
	buf[3] = data_len;
	memcpy(buf +4, data, data_len);
	//buf[4+data_len] = '\0';
	return (4+data_len);
}

static uint8_t * lw_vc0706_form_system_reset(void)
{
	uint8_t *buf;

	buf = vc0706_info.sbuf_info.sbuf;
	vc0706_info.sbuf_info.sbuf_len = lw_vc0706_fill_command(buf, LW_VC0706_FLAG_RECV, 0x00, 
		LW_VC0706_CMD_SYSTEM_RESET, 0x00, NULL);
	return buf;
}

static uint8_t * lw_vc0706_form_stop_cfbuf(void)
{
	uint8_t data;
	uint8_t *buf;
	
	data = LW_VC0706_FBUF_CURRENT_FRAME;
	buf = vc0706_info.sbuf_info.sbuf;
	vc0706_info.sbuf_info.sbuf_len = lw_vc0706_fill_command(buf, LW_VC0706_FLAG_RECV, 0x00,
		LW_VC0706_CMD_FBUF_CTRL, 0x01, &data);
	return buf;
}

static uint8_t * lw_vc0706_form_get_len(void)
{
	uint8_t data;
	uint8_t *buf;

	data = LW_VC0706_FBUF_CURRENT_FRAME;
	buf = vc0706_info.sbuf_info.sbuf;
	vc0706_info.sbuf_info.sbuf_len = lw_vc0706_fill_command(buf, LW_VC0706_FLAG_RECV, 0x00,
		LW_VC0706_CMD_GET_FBUF_LEN, 0x01, &data);
	return buf;
}

static uint8_t read_fbuf_trans_data_mode(bool isDMA /* MCU or DMA*/,
	uint8_t recv_port_type /*0: MCU串口, 1:　高速串口,  2: SPI*/,
	bool isSPIHost);
static uint8_t * lw_vc0706_form_read_fbuf(uint32_t pic_len, uint16_t delay)
{
	uint32_t start_addr;
	uint8_t *buf;

	start_addr = 0;
	buf = vc0706_info.sbuf_info.sbuf;
	buf[0] = LW_VC0706_FLAG_RECV;
	buf[1] = 0x00;
	buf[2] = LW_VC0706_CMD_READ_FBUF;
	buf[3] = 0x0c;
	buf[4] = LW_VC0706_FBUF_CURRENT_FRAME;
	buf[5] = read_fbuf_trans_data_mode(0, 0, 1);
	
	buf[6] = (start_addr >> 24) & 0xff;	
	buf[7] = (start_addr >> 16) & 0xff;		
	buf[8] = (start_addr >> 8) & 0xff;
	buf[9] = start_addr & 0xff;

	buf[10] = (pic_len >> 24) & 0xff;	
	buf[11] = (pic_len >> 16) & 0xff;	
	buf[12] = (pic_len >> 8) & 0xff;
	buf[13] = pic_len & 0xff;
	
	buf[14] = (delay >> 8) & 0xff;
	buf[15] = delay & 0xff;
	
	vc0706_info.sbuf_info.sbuf_len = 16;
	
	return buf;
}



static uint8_t read_fbuf_trans_data_mode(bool isDMA /* MCU or DMA*/,
	uint8_t recv_port_type /*0: MCU串口, 1:　高速串口,  2: SPI*/,
	bool isSPIHost)
{
	uint8_t mode;

	mode = 0x00;
	if(isDMA)
		mode |= 0x01;
	switch (recv_port_type)
	{
		case 0:
			mode |= 0x01 << 1;
			break;
		case 1:
			mode |= 0x10 << 1;
			break;
		case 2:
			mode |= 0x11 <<1;
			break;
		default:
			break;
	}

	if(isSPIHost)
		mode |= 0x01 << 3;

	return mode;
}

#define cam_send_cmd(x, y, z) com_send_nchar(x, y, z)
static void lw_vc0706_send_system_reset(void)
{
	vc0706_info.status = VC0706_STATUS_SYSTEM_RESET;
	cam_send_cmd(3, lw_vc0706_form_system_reset(), vc0706_info.sbuf_info.sbuf_len);
}

static void lw_vc0706_send_stop_cfbuf(void)
{
	vc0706_info.status = VC0706_STATUS_STOP_CFBUF;
	cam_send_cmd(3, lw_vc0706_form_stop_cfbuf(), vc0706_info.sbuf_info.sbuf_len);
}

static void lw_vc0706_send_get_len(void)
{
	vc0706_info.status = VC0706_STATUS_GET_LEN;
	cam_send_cmd(3, lw_vc0706_form_get_len(), vc0706_info.sbuf_info.sbuf_len);
}

static void lw_vc0706_send_read_fbuf(void)
{
	vc0706_info.status = VC0706_STATUS_READ_FBUF;
	cam_send_cmd(3, 
		lw_vc0706_form_read_fbuf(/*0x0000B694*/ vc0706_info.rbuf_info.frame_len, 0x0BB8),
			vc0706_info.sbuf_info.sbuf_len);
}

static void lw_vc0706_trans_finished(void)
{
	vc0706_info.status = VC0706_STATUS_TRANS_FINISHED;
}

void lw_vc0706_get_frame(void)
{
	switch(vc0706_info.status){
		case VC0706_STATUS_NOINIT:
		lw_vc0706_send_system_reset();
		break;
		case VC0706_STATUS_SYSTEM_RESET:
		lw_vc0706_send_stop_cfbuf();
		break;
		case VC0706_STATUS_STOP_CFBUF:
		lw_vc0706_send_get_len();
		break;
		case VC0706_STATUS_GET_LEN:
		lw_vc0706_send_read_fbuf();
		break;
		case VC0706_STATUS_READ_FBUF:
		lw_vc0706_trans_finished();
		break;
		case VC0706_STATUS_TRANS_FINISHED:
		break;
		default:
		break;
	}
}

void test_cam(void)
{
	if(vc0706_info.status == VC0706_STATUS_TRANS_FINISHED)
		;//vc0706_info.status = VC0706_STATUS_NOINIT;
	lw_vc0706_get_frame();
}

static void get_frame_len(uint8_t *buf)
{
	//static uint8_t hex2ascii[] = "0123456789ABCDEF";

/*	uint8_t tmp[4];

	tmp[0] = ((buf[0]>>4) - '0') *16 + ((buf[0]&0xf) - '0');
	tmp[1] = ((buf[1]>>4) - '0') *16 + ((buf[1]&0xf) - '0');
	tmp[2] = ((buf[2]>>4) - '0') *16 + ((buf[2]&0xf) - '0');
	tmp[3] = ((buf[3]>>4) - '0') *16 + ((buf[3]&0xf) - '0');
	vc0706_info.rbuf_info.frame_len = (tmp[0]<<24) | (tmp[1]<<16) | (tmp[2]<<8) |tmp[3];
	*/

	//sscanf(buf, "%4x", &vc0706_info.rbuf_info.frame_len);
	//memcpy(&vc0706_info.rbuf_info.frame_len, buf, 4);

	uint8_t size;
	uint8_t *p;

	size = 4;
	p = (uint8_t *)&vc0706_info.rbuf_info.frame_len;
	while(size-- >0)
		*p++ = buf[size];
	com_send_message(2, "frame_len");
	com_send_hex(2, (vc0706_info.rbuf_info.frame_len>>24) &0xff);
	com_send_hex(2, (vc0706_info.rbuf_info.frame_len>>16) &0xff);	
	com_send_hex(2, (vc0706_info.rbuf_info.frame_len>>8) &0xff);
	com_send_hex(2, vc0706_info.rbuf_info.frame_len &0xff);	
	com_send_message(2, "");
	buf[4] = '\0';
	com_send_nchar(2, buf,4);
	/*com_send_message(2, "");*/
	
}

int32_t lw_cam_get_frame(void)
{
	//VC0706_RXNE_IRQ_ENABLE();
		
	lw_vc0706_send_system_reset();
	delay(10);
	lw_vc0706_send_stop_cfbuf();
	delay(10);		
	VC0706_RXNE_IRQ_DISABLE();
	lw_vc0706_clean_rbuf();
	VC0706_RXNE_IRQ_ENABLE();
	lw_vc0706_send_get_len();
	delay(10);
	get_frame_len(vc0706_info.rbuf_info.rbuf + 5);

	//lw_vc0706_send_read_fbuf();
	//lw_vc0706_trans_finished();

	return 0;
}

int32_t lw_cam_start_frame(void)
{
	uint32_t frame_len = vc0706_info.rbuf_info.frame_len + 10;
	
	VC0706_RXNE_IRQ_DISABLE();

frame_len = 24;
	lw_send_cam_cmd_L(frame_len);
	lw_grps_send_cam_frame_len(frame_len);
	delay(20);
	lw_cam2gprs_set_dma(frame_len);
	lw_cam2gprs_dma_enable();
	lw_vc0706_send_read_fbuf();

	return 0;
}

int32_t lw_cam_stop_frame(void)
{
	if(lw_cam2gprs_is_finished() == 0 )
	{
		lw_cam2gprs_dma_disable();
		return 0;
	}

	return -1;
}