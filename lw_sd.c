#include <string.h>

#include "util.h"
#include "sd_cmd.h"
#include "sd_conf.h"
#include "lw_sd.h"
#include "lw_stm32_spi.h"
#include "lw_stm32_uart.h"

#define TIMEOUT 9000

sd_struct sd_info;

uint8_t sd_get_crc6(uint8_t cmd, uint8_t *param)
{   
    uint8_t i, j;
    uint8_t reg = 0;
    uint8_t array[5];

    array[0] = cmd;
    for (i = 1; i < 5; i++)             /* 将参数的顺序重新排列 */
        array[i] = param[4 - i];
    
    for (i = 0; i < 5; i++)             /* 计算5个字节的CRC7 */
    {
        for (j = 0; j < 8; j++)
        {
            reg <<= 1;
            reg ^= ((((array[i] << j) ^ reg) & 0x80) ? 0x9 : 0);
        }
    }

    return ((reg << 1) + 0x01) ;            /* 计算结果的CRC7左移一位,并将最低位置1 */
}

uint16_t const CRCTable[256]={0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
                            0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
                            0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
                            0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
                            0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
                            0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
                            0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
                            0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
                            0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
                            0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
                            0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
                            0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
                            0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
                            0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
                            0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
                            0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
                            0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
                            0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
                            0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
                            0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
                            0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
                            0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
                            0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
                            0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
                            0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
                            0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
                            0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
                            0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
                            0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
                            0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
                            0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
                            0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
                           };

uint16_t sd_get_crc16(uint8_t *buf, uint32_t buflen)
{  
    uint16_t i;
    uint16_t result = 0;

    for (i = 0; i < buflen; i++)
        result = (result << 8) ^ (uint16_t)CRCTable[(result >> 8) ^ *buf++];

    return result;

}

static uint8_t spi_send_cmd(uint8_t cmd, uint8_t *param, uint8_t resptype, uint8_t *resp)
{
	uint8_t i;
	uint8_t ret;
	uint8_t crc;
	uint8_t rlen;
	uint16_t timeout_cnt=0;

	if (false == sd_cs_low()) {
		com_send_message(2, "send_cmd low err");
		return SD_ERR_CMD_TIMEOUT;
	}
	
	spi_send_char((cmd & 0x3f) | 0x40);

	for(i = 4; i > 0; i--)
		spi_send_char(param[i-1]);
/*	spi_send_char(param[0]);
	spi_send_char(param[1]);
	spi_send_char(param[2]);
	spi_send_char(param[3]);*/

	/* CRC */
	crc = sd_get_crc6((cmd & 0x3f) | 0x40, param);
	spi_send_char(crc);

	switch(resptype) {
	case R1: 
		rlen = 1; 
		break;
	case R1B: 
		rlen = 1; 
		break;
	case R2: 
		rlen = 2; 
		break;
	case R3:
	case R7:
		rlen = 5; 
		break;
	default:
		rlen = 5;
		break;
	}
	
	do
	{
		//com_send_message(1, "111116");	
		ret = spi_recv_char();
		//com_send_message(1, "111117:");	
		//com_send_hex(1, ret);
		//com_send_message(1, "");
		//com_send_hex(1, timeout_cnt);
		//com_send_message(1, "");
		//debug_printf_h(ret);
		//debug_printf_s(",");
		timeout_cnt ++;
	}while(((ret & 0x80) != 0) && (timeout_cnt < TIMEOUT));

	if(timeout_cnt >= TIMEOUT)
	{
		/* FIXME: */
		sd_cs_high();
		return SD_ERR_CMD_TIMEOUT;
	}
	else
	{
		for(i=0; i<rlen; i++)
		{
			//resp[rlen - i - 1] = ret;
			resp[i] = ret;
			ret = spi_recv_char();	/* 最后再发8clock */
		}
	}

	sd_cs_high();
	return SD_NO_ERR;
}

static uint8_t spi_send_cmd_nocs(uint8_t cmd, uint8_t *param, uint8_t resptype, uint8_t *resp)
{
	uint8_t i;
	uint8_t ret;
	uint8_t crc;
	uint8_t rlen;
	uint32_t timeout_cnt=0;
	
	spi_send_char((cmd & 0x3f) | 0x40);

	for(i = 4; i > 0; i--)
		spi_send_char(param[i-1]);
/*	spi_send_char(param[0]);
	spi_send_char(param[1]);
	spi_send_char(param[2]);
	spi_send_char(param[3]);*/

	/* CRC */
	crc = sd_get_crc6((cmd & 0x3f) | 0x40, param);
	spi_send_char(crc);

	switch(resptype) {
	case R1: 
		rlen = 1; 
		break;
	case R1B: 
		rlen = 1; 
		break;
	case R2: 
		rlen = 2; 
		break;
	case R3:
	case R7:
		rlen = 5; 
		break;
	default:
		rlen = 5;
		break;
	}
	
	do
	{
		//com_send_message(1, "111116");	
		ret = spi_recv_char();
		//com_send_message(1, "111117:");	
		//com_send_hex(1, ret);
		//com_send_message(1, "");
		//com_send_hex(1, timeout_cnt);
		//com_send_message(1, "");
		//debug_printf_h(ret);
		//debug_printf_s(",");
		timeout_cnt ++;
	}while(((ret & 0x80) != 0) && (timeout_cnt < 65000 /*TIMEOUT*/));

	if(timeout_cnt >= TIMEOUT)
	{
		/* FIXME: */
		//sd_cs_high();
		return SD_ERR_CMD_TIMEOUT;
	}
	else
	{
		for(i=0; i<rlen; i++)
		{
			//resp[rlen - i - 1] = ret;
			resp[i] = ret;
			if ((i+1) < rlen)
				ret = spi_recv_char();	/* 最后再发8clock */
		}
	}

	return SD_NO_ERR;
}

static void spi_delay(uint8_t val)
{
	while(val -- >0)
		spi_send_char(0xFF);
}

static bool  lw_sd_enable_crc(bool iscrc)
{
	uint8_t ret;
	uint8_t resp;	
	uint8_t param[4]={0,0,0,0};

	if(true == iscrc)
		param[0] = 1;
	else
		param[0] = 0;

	ret = spi_send_cmd(CMD59, param, CMD59_R, &resp);
	if(ret != SD_NO_ERR)
	{
		return false;
	}
	else if(resp != 0)
	{
		return false; //SD_ERR_CMD_RESP;
	}

	return true; //SD_NO_ERR;
}

static void sd_pack_param(uint8_t *param, uint32_t value)
{
	param[3] = (uint8_t)(value >> 24);
	param[2] = (uint8_t)(value >> 16);
	param[1] = (uint8_t)(value >> 8);
	param[0] = (uint8_t)(value);
}   
	
static bool lw_sd_set_block_length(uint32_t block_length)
{
	uint8_t ret;
	uint8_t resp;	
	uint8_t param[4]={0,0,0,0};

	uint16_t timeout = 500;
	uint16_t i=0;

	do {
	sd_pack_param(param, block_length);
	ret = spi_send_cmd(CMD16, param, CMD16_R, &resp);
	if(ret != SD_NO_ERR) {
		com_send_message(2, "CMD16 cmd err");
		//return false;
	}

	/*  >= 2GB　的卡返回resp=0x40,表示不认识这个命令，默认就是512BYTE的块*/
	/*
	else if( resp != 0) {
		com_send_message(2, "CMD16 resp err");
		com_send_hex(2, resp);
		com_send_message(2, "");
		//return false; //SD_ERR_CMD_RESP;
	}
	}while(resp && (i++ < timeout));*/
	else if( resp & (~0x40)) {
		com_send_message(2, "CMD16 resp err");
		//return false; //SD_ERR_CMD_RESP;
	}		
	}while((resp & (~0x40)) && (i++ < timeout));

	if (i >= timeout)
		return false;
	
	return true; //SD_NO_ERR;
}
int32_t lw_sd_init(void)
{
	uint8_t ret;
	uint8_t resp;	
	uint8_t param[4]={0,0,0,0};

	sd_cs_high();
	//delay(100);
	for (ret=0; ret<80; ret++)
		spi_send_char(0xFF);
	
	if (false == sd_cs_low()) {
		com_send_message(2, "sd_init low err");
		return SD_ERR_CMD_TIMEOUT;
	}
	//spi_delay(200);

	/*sd_cs_high();
	//sd_cs_low();
	spi_delay(200);
	sd_cs_high();
	//delay(100);*/
	if((ret = spi_send_cmd(CMD0, param, CMD0_R, &resp)) != SD_NO_ERR)
	{
		com_send_message(2, "CMD0 ret err");
		return ret;
	}
	else if (resp != 0x01)
	{
		com_send_message(2, "CMD0 resp err");
		return SD_ERR_CMD_RESP;
	}
	//com_send_message(2, "CMD0 OK");
	//com_send_char(1, val);

	return SD_NO_ERR;

}

static bool sd_get_card_info(void)
{
#if 0
    uint32_t tmp;
    uint8_t csdbuf[16],ret;

    ret = SD_ReadCSD(16, csdbuf);                                               /* 读CSD寄存器    read CSD register */
    if (ret != SD_NO_ERR)
        return ret; 
        
    SD_CalTimeout(csdbuf);                                                      /* 计算超时时间值 calculate timeout value */
        
    /* 计算块的最大长度  */                                                     /* calculate the size of a sector */
    sds.block_len = 1 << (csdbuf[READ_BL_LEN_POS] & READ_BL_LEN_MSK);           /* (2 ^ READ_BL_LEN) */

    /* 计算卡中块的个数 */                                                      /* calculate the sector numbers of the SD Card */
    sds.block_num = ((csdbuf[C_SIZE_POS1] & C_SIZE_MSK1) << 10) +
                     (csdbuf[C_SIZE_POS2] << 2) +
                    ((csdbuf[C_SIZE_POS3] & C_SIZE_MSK3) >> 6) + 1;             /* (C_SIZE + 1)*/
                          
    tmp = ((csdbuf[C_SIZE_MULT_POS1] & C_SIZE_MULT_MSK1) << 1) +
          ((csdbuf[C_SIZE_MULT_POS2] & C_SIZE_MULT_MSK2) >> 7) + 2;             /* (C_SIZE_MULT + 2) */

    /* 获得卡中块的数量 */                                                      /* get the block numbers in card */
    sds.block_num = sds.block_num * (1 << tmp);                                 /* (C_SIZE + 1) * 2 ^ (C_SIZE_MULT + 2) */
    /* 计算擦除的单位(单位: 块) */
    if (sds.card_type == CARDTYPE_MMC)
    {
        tmp  = ((csdbuf[ERASE_GRP_SIZE_POS] & ERASE_GRP_SIZE_MSK) >> 2) + 1;    /* (ERASE_GRP_SIZE + 1)  */

        /* (ERASE_GRP_SIZE + 1) * (ERASE_GRP_MULTI + 1) */
        tmp *= ((csdbuf[ERASE_GRP_MULTI_POS1] & ERASE_GRP_MULTI_MSK1) << 3) +
               ((csdbuf[ERASE_GRP_MULTI_POS2] & ERASE_GRP_MULTI_MSK2) >> 5) + 1;
    }
    else                                                                        /*calculate the size of sector */
        tmp = ((csdbuf[SECTOR_SIZE_POS1] & SECTOR_SIZE_MSK1) << 1) +
              ((csdbuf[SECTOR_SIZE_POS2] & SECTOR_SIZE_MSK2) >> 7) + 1;         /* SD: SECTOR_SIZE */

    sds.erase_unit = tmp;                                                       /* 擦除单位(块) */
#endif
    return true;//SD_NO_ERR;                                                           /* 返回执行成功 return perform sucessfully */
}


static bool lw_sd_send_CMD1(void)
{
	uint8_t ret;
	uint8_t param[4]={0,0,0,0};
	uint8_t resp[5];
	uint32_t i;

	i = 0;
	do {
		/* CMD1只用于MMC?? */
		ret = spi_send_cmd(CMD1, param, CMD1_R, resp);	/*FIXME:断电重启后第一次总会不成功*/
		if(ret != SD_NO_ERR) {

		}
		i++;
		//delay(5);
	}while(((resp[0] & MSK_IDLE) == MSK_IDLE) && (i < SD_IDLE_WAIT_MAX)) ;

	if(i >= SD_IDLE_WAIT_MAX) {
		return false; //com_send_message(2, "sd_active CMD1 err");
	}
	else
		return true; //com_send_message(2, "sd_active CMD1 ok");
}

static bool lw_sd_send_CMD8(void)
{
	uint8_t ret;
	uint8_t param[4]={0,0,0,0};
	uint8_t resp[5];
	uint32_t i;

	sd_pack_param(param, 0x100 /* 2.7V - 3.6V */ | 0xaa /* test pattern */);
	
	i = 0;
	//do {
		/* verify SD Memory Card interface operating condition */
		ret = spi_send_cmd(CMD8, param, CMD8_R, resp); 
		if(ret != SD_NO_ERR){
			return false;
		}
		i++;
	/*}while(((resp[0] & MSK_IDLE) == MSK_IDLE) && (i < SD_IDLE_WAIT_MAX)) ;
	if(i >= SD_IDLE_WAIT_MAX) {
		return false; //com_send_message(2, "sd_active CMD1 err");
	}
	else
		return true; //com_send_message(2, "sd_active CMD1 ok");
		*/


	if (resp[0] & 0x04) { /* V1 or not SD card */
		com_send_message(2, "v1 or not sd card");
		/*FIXME:针对v1的相应处理*/
	}

	//if (resp[0] == 0x01) { /* V2 card */
	//}	
	com_send_message(2, "v2 card");
	/*FIXME:针对v2的相应处理*/

	return true;
}

static bool lw_sd_send_ACMD41(void)
{
	uint8_t ret;
	uint8_t param[4]={0,0,0,0};
	uint8_t resp;

	uint16_t timeout_cnt = 1000;
	uint16_t i=0, j=0;

do {
	do {
	ret = spi_send_cmd(CMD55, param, CMD55_R, &resp);
	if(ret != SD_NO_ERR) {
		com_send_message(2, "acmd41->cmd55 cmd err");
		return false;
	}
	else if (resp & 0x04) {
		com_send_message(2, "acmd41->cmd55 resp err");
		com_send_hex(2,resp);		
		//return false;		
	}
/*	else if (resp != 0) {
		com_send_message(2, "acmd41->cmd55 resp err");
		com_send_hex(2,resp);		
		return false;
	}*/
	} while((resp & 0x04) && (i++ < timeout_cnt));

	/* FIXME:bit30(HCS) 是否置位???? */
	ret = spi_send_cmd(ACMD41, param, ACMD41_R, &resp);
	if(ret != SD_NO_ERR) {
		com_send_message(2, "acmd41 cmd err");
		return false;
	}

	if((resp & 0x04) == 0) {
		sd_info.card_type = CARDTYPE_SD;
		com_send_message(2, "is SD card");
	}
	else {
		sd_info.card_type = CARDTYPE_MMC;
		com_send_message(2, "is mmc card");
	}
}while((resp & 0x01) && (j++ < timeout_cnt));

	return true;
}

int32_t lw_sd_active(void)
{
	bool ret;

	ret = lw_sd_send_CMD1();
	if (ret == false)
		com_send_message(2, "sd_active CMD1 err");
	else
		com_send_message(2, "sd_active CMD1 ok");


	ret = lw_sd_send_CMD8();
	if (ret == false)
		com_send_message(2, "sd_active CMD8 err");
	else
		com_send_message(2, "sd_active CMD8 ok");
	
	ret = lw_sd_send_ACMD41();
	if (ret == false)
		com_send_message(2, "sd_active ACMD41 err");
	else
		com_send_message(2, "sd_active ACMD41 ok");
		
	
	ret = lw_sd_enable_crc(false);
	if(ret == false)
		com_send_message(2, "sd_active CRC err");
	else
		com_send_message(2, "sd_active CRC ok");

	ret = lw_sd_set_block_length(SD_BLOCK_SIZE);
	if(ret == false)
		com_send_message(2, "sd_active SET_BLOCK_LEN err");
	else
		com_send_message(2, "sd_active SET_BLOCK_LEN ok");
  
	ret = sd_get_card_info();/* 9. 读CSD寄存器,获取SD卡信息 read CSD register, get the information of SD card */   
	if(ret == false)
		;//com_send_message(2, "sd_active SET_BLOCK_LEN err");
	else
		;//com_send_message(2, "sd_active SET_BLOCK_LEN ok");
  
    
	//SD_EndSD(); 

	sd_spi_clk_tomax();                                 /* 10. 设置SPI时钟到最大值 set SPI clock to maximum */

    return SD_NO_ERR;                               /* 初始化成功 initialize sucessfully */
}
/*
	cmdtype: 	0:写单个块
				1:写多个块
				2:读单个块
				3:读多个块
*/
static int32_t lw_sd_write_block(uint8_t cmdtype, uint32_t addr)
{
	uint8_t resp;
	int32_t ret;
	uint8_t param[4];

	sd_pack_param(param, addr);
	switch (cmdtype) {
	case 0:
		//ret = spi_send_cmd(CMD24, param, CMD24_R, &resp);
		ret = spi_send_cmd_nocs(CMD24, param, CMD24_R, &resp);
		break;
	case 1:
		//ret = spi_send_cmd(CMD25, param, CMD25_R, &resp);
		ret = spi_send_cmd_nocs(CMD25, param, CMD25_R, &resp);
		break;
	case 2:
		//ret = spi_send_cmd(CMD17, param, CMD17_R, &resp);
		ret = spi_send_cmd_nocs(CMD17, param, CMD17_R, &resp);
		break;
	case 3:
		//ret = spi_send_cmd(CMD18, param, CMD18_R, &resp);
		ret = spi_send_cmd_nocs(CMD18, param, CMD18_R, &resp);
		break;
	default:
		ret = SD_ERR_CMD_TIMEOUT;
		break;
	}
	
	if(ret != SD_NO_ERR) {
		com_send_message(2, "write_block cmd err");
		debug_printf_s("errno ret=");
		debug_printf_h(ret);
		debug_printf_s("");
		return ret;
	}
	//else if (resp & 0x80) { /*?????? 参考ffsample只要看高位*/
	else if( resp != 0) {
		com_send_message(2, "write_block resp err:");
		com_send_hex(2, resp);
		com_send_message(2, "");
		return SD_ERR_CMD_RESP;
	}
	
	return SD_NO_ERR;
}

int32_t lw_sd_wait_busy(uint8_t waittype)
{
	uint8_t ret;
	uint32_t timeout_cnt;
	uint32_t i;

	timeout_cnt = 65000;
	i = 0;
/*	if (false == sd_cs_low()) {
		com_send_message(2, "sd_wait_busy low err");
		return SD_ERR_CMD_TIMEOUT;
	}*/
	do {
		ret = spi_recv_char();
		//i++;
	//}while((ret != 0xFF) && (i < timeout_cnt));
	}while(ret != 0xFF) ;

	//spi_send_char(0xFF);
	/*sd_cs_high();*/

	if (i < timeout_cnt)
		return SD_NO_ERR;
	return SD_ERR_TIMEOUT_WAIT;
}

static int32_t lw_sd_write_data(bool mult_block, uint8_t *buf, uint32_t buflen)
{
	uint32_t i;
	uint16_t crc;
	uint32_t timeout_cnt=0;

/*	if (false == sd_cs_low()) {
		com_send_message(2, "sd_write_data low err");
		return SD_ERR_CMD_TIMEOUT;
	}*/
		

	spi_send_char(0xFF);

	if (mult_block == 1)
		spi_send_char(SD_TOK_WRITE_STARTBLOCK_M);/* 写多块开始令牌 start token of write multi blocks */
	else
		spi_send_char(SD_TOK_WRITE_STARTBLOCK);  /* 写单块开始令牌 start token of write single block */

	for(i = 0; i < buflen; i++)
		spi_send_char(buf[i]);

	for(i=0; i<SD_BLOCK_SIZE-buflen; i++)
		spi_send_char(0x00);

	crc = sd_get_crc16(buf, buflen);
	spi_send_char((crc>>8) & 0xFF);
	spi_send_char(crc & 0xFF);

#if 0
	ret = 0x00;
	while (0x00 == ret) {	/* busy */
		ret = spi_recv_char();
	}
#endif

	//sd_cs_high();

	#if 0
	if ((spi_recv_char() & 0x1f) != 0x05) { /*???????参考ffsample*/
		com_send_message(2, "err not accept data");
		return SD_ERR_TIMEOUT_WRITE;
	}
	#else

	if (lw_sd_wait_busy(SD_WAIT_WRITE) != SD_NO_ERR) {
		com_send_message(2, "write block timeout");
//		sd_cs_high();
		return SD_ERR_TIMEOUT_WRITE;
	}
	#endif

//	sd_cs_high();

	return SD_NO_ERR;

}

static int32_t lw_sd_read_data(bool mult_block, uint8_t *buf, uint32_t buflen)
{
	uint8_t val;
	uint32_t timeout_cnt;
	uint32_t i;

	timeout_cnt = 9000;
/*	if (false == sd_cs_low()) {
		com_send_message(2, "sd_read_data low err");
		return SD_ERR_CMD_TIMEOUT;
	}*/
	
	do {
		val = spi_recv_char();
		i++;
		//com_send_hex(2, val);
		//com_send_message(2, "");
	}while((val != SD_TOK_READ_STARTBLOCK) && (i < timeout_cnt ));

	if(i >= timeout_cnt) {
//		sd_cs_high();
		com_send_message(2, "read_data cmd err");
		return SD_ERR_TIMEOUT_READ;
	}

/*	if (val != SD_TOK_READ_STARTBLOCK) {
		spi_send_char(0xFF);
		sd_cs_high();
		return SD_ERR_DATA_START_TOK;
	}*/

	for(i=0; i<buflen; i++) {
		buf[i] = spi_recv_char();
	}
	
	for (i=0; i< SD_BLOCK_SIZE - buflen; i++)
		spi_recv_char();

	i = spi_recv_char();				/* CRC */
	i = (i<<8) + spi_recv_char();
/*	if ( i != sd_get_crc16(buf, buflen)) {
		spi_send_char(0xFF);
		sd_cs_high();
		return SD_ERR_DATA_CRC16;
	}*/

/*	spi_send_char(0xFF);
	sd_cs_high();*/
	return SD_NO_ERR;
}

static int32_t lw_sd_check_write_ok()
{
	uint8_t resp[2];
	int32_t ret;
	uint8_t param[]={0, 0, 0, 0};

	//debug_printf_m("will CMD13:");
	ret = spi_send_cmd_nocs(CMD13, param, CMD13_R, resp);
	if (ret != SD_NO_ERR) {
		com_send_message(2, "CMD13 cmd err");		
		return ret;
	}

	if ((resp[0] !=0) || (resp[1] != 0)) {
		com_send_message(2, "CMD13 resp err:");
		com_send_hex(2, resp[0]);
		com_send_string(2, ",");
		com_send_hex(2, resp[1]);
		com_send_message(2, "");
		return SD_ERR_WRITE_BLK;
	}
	
	return SD_NO_ERR;
}

int32_t _lw_sd_write(uint8_t *buf, uint32_t buflen, uint32_t addr)
{
	int32_t ret;
/*
debug_printf_s("addr=");
debug_printf_h(addr>>24);
debug_printf_s(",");
debug_printf_h(addr>>16);
debug_printf_s(",");
debug_printf_h(addr>>8);
debug_printf_s(",");
debug_printf_h(addr>>0);
debug_printf_m("");
*/
	sd_cs_low();

	ret = lw_sd_write_block(0, addr);
sd_cs_high();	
	if(ret != SD_NO_ERR) {
		com_send_message(2, "lw_sd_write_block err");
		goto err; //return ret;
	}
sd_cs_low();
	ret = lw_sd_write_data(0, buf, buflen);
	if (ret != SD_NO_ERR) {
		com_send_message(2, "lw_sd_write_data err");
		goto err; //return ret;
	}
sd_cs_high();
sd_cs_low();

	/*??????参考ffsample下面这句可以不用的*/
	#if 0
	ret = lw_sd_check_write_ok();
	if (ret != SD_NO_ERR) {
		com_send_message(2, "lw_sd_check_write_ok err");
		goto err; //return ret;
	}
	#endif

	sd_cs_high();
	
	return SD_NO_ERR;

err:
	sd_cs_high();	
	return ret;
}

int32_t _lw_sd_read(uint8_t *buf, uint32_t buflen, uint32_t addr)
{
	int32_t ret;

	sd_cs_low();


	ret = lw_sd_write_block(2, addr);
	if(ret != SD_NO_ERR) {
		com_send_message(2, "read->write_block err");
		return ret;
	}

	ret = lw_sd_read_data(0, buf, buflen);
	if(ret != SD_NO_ERR) {
		com_send_message(2, "read->read_data err");
		return ret;
	}

	sd_cs_high();

	return SD_NO_ERR;
}

int32_t lw_sd_write(uint8_t *buf, uint32_t buflen)
{
	int8_t i;
	uint8_t blocks;
	uint32_t blocknum;
	uint32_t addr;
	int32_t ret;
	//uint8_t resp[];

	blocknum = 0x265;//8;
	addr = blocknum * SD_BLOCK_SIZE;
	//return (_lw_sd_write(buf, buflen, addr));

	blocks = buflen/SD_BLOCK_SIZE + ((buflen%SD_BLOCK_SIZE) ? 1:0);
	for (i=0; i< blocks; i++) {
		if (i < blocks - 1)
			ret = _lw_sd_write(buf + i*SD_BLOCK_SIZE, SD_BLOCK_SIZE, addr + i*SD_BLOCK_SIZE);
		else
			ret = _lw_sd_write(buf + i*SD_BLOCK_SIZE, buflen%SD_BLOCK_SIZE, addr + i*SD_BLOCK_SIZE);
		
		if (ret != SD_NO_ERR)
			return ret;
	}

	return SD_NO_ERR;
}

int32_t lw_sd_read(uint8_t *buf, uint32_t buflen)
{
	int8_t i;
	uint8_t blocks;
	uint32_t blocknum;
	uint32_t addr;
	int32_t ret;

	blocknum = 0x265;//8;
	addr = blocknum * SD_BLOCK_SIZE;
	//return (_lw_sd_read(buf, buflen, addr));

	blocks = buflen/SD_BLOCK_SIZE + ((buflen%SD_BLOCK_SIZE) ? 1:0);
	for (i=0; i< blocks; i++) {
		if (i < blocks - 1)
			ret = _lw_sd_read(buf + i*SD_BLOCK_SIZE, SD_BLOCK_SIZE, addr + i*SD_BLOCK_SIZE);
		else
			ret = _lw_sd_read(buf + i*SD_BLOCK_SIZE, buflen%SD_BLOCK_SIZE, addr + i*SD_BLOCK_SIZE);
		
		if (ret != SD_NO_ERR)
			return ret;
	}	

	return SD_NO_ERR;
}

	uint8_t sd_test_buf[1025];//="12abc";
	uint8_t sd_rbuf[1025];
void lw_sd_test(void)
{
	uint32_t sd_test_buflen;
	int32_t ret;

	int8_t test_val = 0;

	for (ret=0; ret<1024; ret++) {
		sd_test_buf[ret] = 'a' + test_val;
		if (++test_val >= 26)
			test_val = 0;
	}

	sd_test_buflen = SD_BLOCK_SIZE +1;//strlen(sd_test_buf);
	
	ret = lw_sd_init();
	if(ret != SD_NO_ERR)
		com_send_message(2, "lw_sd_init err");
	else
		com_send_message(2, "lw_sd_init ok");

	lw_sd_active();	
	
	while(1) {
/*		sd_test_buf[0] = test_val + '0';
		if (++test_val > 9)
			test_val = 0;*/
		
		if (lw_sd_write(sd_test_buf, sd_test_buflen) != SD_NO_ERR)
			com_send_message(2, "write sd err");
		else
			com_send_message(2, "write sd ok");
		//continue;
		memset(sd_rbuf, 0, sd_test_buflen);
		if (lw_sd_read(sd_rbuf, sd_test_buflen))
			com_send_message(2, "read sd err");
		else {
			com_send_message(2, "read sd ok");
			sd_rbuf[sd_test_buflen] = '\0';
			com_send_message(2, "read:");
			com_send_message(2, sd_rbuf);
		}
	}
}


int32_t lw_sd_disk_initialize(void)
{
	int32_t ret;
	
	ret = lw_sd_init();
	if(ret != SD_NO_ERR) {
		com_send_message(2, "lw_sd_init err");
		return -1;
	}
	else
		com_send_message(2, "lw_sd_init ok");

	ret = lw_sd_active();	
	if (ret != SD_NO_ERR) {
		com_send_message(2, "lw_sd_active err");
		return -1;
	}
	else 
		com_send_message(2, "lw_sd_active ok");

	return 0;
}

int32_t lw_sd_disk_status(void)
{
	int32_t ret;
	
	/* FIXME: */
	sd_cs_low();
	ret = lw_sd_check_write_ok();
	sd_cs_high();

	return ret;
	//return 0;
}

int32_t lw_sd_disk_read(uint8_t *buf, uint32_t start_sector, uint32_t count)
{
	uint32_t addr;

	addr = start_sector * SD_BLOCK_SIZE;
	while(count--) {
		if (_lw_sd_read(buf, SD_BLOCK_SIZE, addr))
			return -1;
		buf += SD_BLOCK_SIZE;
		addr += SD_BLOCK_SIZE;
	}

	return 0;
}


int32_t lw_sd_disk_write(uint8_t *buf, uint32_t start_sector, uint32_t count)
{
	uint32_t addr;

	addr = start_sector * SD_BLOCK_SIZE;
	while(count--) {
		if (_lw_sd_write(buf, SD_BLOCK_SIZE, addr))
			return -1;
		buf += SD_BLOCK_SIZE;
		addr += SD_BLOCK_SIZE;
	}

	return 0;
}

int32_t lw_sd_disk_power_off(void)
{
	return 0;
}

int32_t lw_sd_disk_power_on(void)
{
	return 0;
}

int32_t lw_sd_disk_power_sta(void)
{
	return 1; /*0:off, 1:on*/
}

int32_t lw_sd_disk_ctrl_sync(void)
{
	sd_cs_low();
	
	if (lw_sd_wait_busy(SD_WAIT_WRITE)) {
		sd_cs_high();
		return -1;
	}

	sd_cs_high();
	return 0;
}

int32_t lw_sd_disk_get_sector_count(uint32_t *buf)
{
	*buf = (2UL)*1000*1000*1000/SD_BLOCK_SIZE;	
	return 0;
}

int32_t lw_sd_disk_get_sector_size(uint32_t *buf)
{
	*buf = SD_BLOCK_SIZE;
	return 0;
}

int32_t lw_sd_disk_get_blokc_size(uint32_t *buf)
{
	if (1)  { /*V2 card*/
		/*ACMD13*/
		*buf = SD_BLOCK_SIZE;
	}
	else {
		/*CMD9*/
	}

	return 0;
}

int32_t lw_sd_disk_get_type(uint8_t *buf)
{
	*buf = 0x04; /* sd v2*/
	return 0;
}

int32_t lw_sd_disk_get_csd(uint8_t *buf)
{
	/*CMD9*/ /*return buf 16bytes*/
	return 0;
}

int32_t lw_sd_disk_get_cid(uint8_t *buf)
{
	/*CMD10*/ /*return buf 16bytes*/
	return 0;
}

int32_t lw_sd_disk_get_ocr(uint8_t *buf)
{
	/*CMD58*/ /*return buf 4bytes*/
	return 0;
}

int32_t lw_sd_disk_get_sdstat(uint8_t *buf)
{
	/*CMD13*/ /*return buf 64bytes*/
	return 0;
}

uint32_t lw_sd_disk_get_fattime(void)
{
	uint32_t time;
	
	/* FIXME: should from GPS UTC time */
    return    ((uint32_t)(2012 - 1980) << 25)  /* Year = 2012 */
            | ((uint32_t)1 << 21)              /* Month = 1 */
            | ((uint32_t)1 << 16)              /* Day_m = 1*/
            | ((uint32_t)0 << 11)              /* Hour = 0 */
            | ((uint32_t)0 << 5)               /* Min = 0 */
            | ((uint32_t)0 >> 1);              /* Sec = 0 */

	
	return time;
}

#include "ff.h"

FATFS Fatfs;        /* File system object */
FIL Fil;            /* File object */
uint8_t wBuff[512];     /* File read buffer */
uint8_t rBuff[512];

int32_t lw_sd_fatfs_init(void)
{
	FRESULT rc;             /* Result code */
	
	rc = f_mount(0, &Fatfs);     /* Register volume work area (never fails) */
	if (rc ) {
		debug_printf_m("mount err");
		return -1;
	}

	return 0;
}

int32_t lw_sd_fatfs_deinit(void)
{
	FRESULT rc;             /* Result code */
	
	rc = f_mount(0, NULL);     /* Unregister volume work area (never fails) */	
	if (rc ) {
		debug_printf_m("umount err");
		return -1;
	}

	return 0;
}

int32_t lw_get_frame2sd(void)
{
	#include "lw_vc0706.h"
	
	bool finish;
	
	uint8_t *p;
	uint16_t retry;
	uint16_t file_faile_timeout;
	uint32_t bw;
	uint32_t len;
	FIL Fil;            /* File object */
	FRESULT rc;             /* Result code */
	static uint32_t file_num = 1;
	uint8_t file_name[16];

	uint64_t old_pz;
	uint32_t cnt = 0;
	
	if (lw_cam_get_frame()) {
		debug_printf_m("get frame len err");
		return -2;
	}

	if (lw_sd_fatfs_init())
		return -1;
	sprintf(file_name, "c%d.jpg", file_num++);
	if (f_open(&Fil, file_name, FA_WRITE | FA_CREATE_ALWAYS)) {
		if (f_open(&Fil, file_name, FA_WRITE | FA_CREATE_ALWAYS)) {
			debug_printf_s("open ");
			debug_printf_s(file_name);
			debug_printf_s( " err");
			debug_printf_m("");
			//debug_printf_m("open cam.jpg err");
			f_close(&Fil);
			lw_sd_fatfs_deinit();
			return -1;
		}
	}

	lw_cam_start_frame_();

	
	
	do{
		old_pz = f_tell(&Fil);
		finish = lw_get_cam_data(&p, &len);
		if (len) {
			cnt += len;
			retry = 0;
			do {
				rt:
				rc = f_write(&Fil, p, len, &bw);
				if (rc) {
					debug_printf_m("write err");
					if (retry ++ < 5) {
						debug_printf_m("retry");
#if 0
						f_close(&Fil);
						if (f_open(&Fil, file_name, FA_WRITE)) {
							debug_printf_m("retry open err");
							f_close(&Fil);
							lw_sd_fatfs_deinit();
							return -1;
						}
#endif						
						f_lseek(&Fil, old_pz);
						goto rt;
					}
					f_close(&Fil);
					lw_sd_fatfs_deinit();
					//file_num--;
					return -1;
				}
				if (len -= bw) {
					debug_printf_m("warring len!=bw");
					p += bw;
				}
			}while (len);
		}
	}while (false == finish);

	lw_cam_stop_frame_();
	
	debug_printf_m("");
	debug_printf_s("save to sd len=");
	debug_printf_h(cnt>>24);
	debug_printf_s(",");
	debug_printf_h(cnt>>16);
	debug_printf_s(",");
	debug_printf_h(cnt>>8);
	debug_printf_s(",");	
	debug_printf_h(cnt);
	debug_printf_m("");
	
	if (f_close(&Fil)) {
		debug_printf_m("close cam.jpg err");
		lw_sd_fatfs_deinit();
		return -1;
	}

	lw_sd_fatfs_deinit();
	return 0;
}

void lw_sd_with_fatfs_test(void)
{
	FRESULT rc;             /* Result code */
	DIR dir;                /* Directory object */
	FILINFO fno;            /* File information object */
	uint32_t bw, br, i;

	static uint8_t s_val = 0;
	
	rc = f_mount(0, &Fatfs);     /* Register volume work area (never fails) */
	if (rc ) debug_printf_m("mount err");
	
	debug_printf_m("\nOpen an existing file (message.txt).\n");
	rc = f_open(&Fil, "MESSAGE.TXT", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) {
		debug_printf_s("open err, rc=");
		debug_printf_h(rc);
		debug_printf_m("");
		return ;
	}

	debug_printf_m("\nType the file content.\n");

	bw = 0;
	for (i=0; i<512; i++) {
		wBuff[i]=bw + 'A';
		if (++bw >= 26)
			bw = 0;
	}
	wBuff[1] = s_val++;
	debug_printf_m("will write");
	for (;;) {
		rc = f_write(&Fil, wBuff, 512, &bw);
		if (rc || bw) break;
	}
	if (rc) {
		debug_printf_m("write err");
//		return ;
	}

	debug_printf_m("\nClose the file.\n");
	rc = f_close(&Fil);



	#if 1
	rc = f_open(&Fil, "MESSAGE.TXT", FA_READ);
	if (rc) {
		debug_printf_s("open err, rc=");
		debug_printf_h(rc);
		debug_printf_m("");
		return ;
	}
	
	debug_printf_m("will read");
	for (;;) {
	    rc = f_read(&Fil, rBuff, sizeof rBuff, &br);  /* Read a chunk of file */
	    if (rc || !br) break;           /* Error or end of file */
	    for (i = 0; i < br; i++)        /* Type the data */
	    {
		debug_printf_h(rBuff[i]);
		debug_printf_s(",");
		}
	}
	if (rc) {
		debug_printf_m("read err");
//		return ;
	}

	debug_printf_m("\nClose the file.\n");
	rc = f_close(&Fil);
	#endif
	
	rc = f_mount(0, NULL);     /* Unregister volume work area (never fails) */	

#if 0
	FRESULT rc;             /* Result code */
	DIR dir;                /* Directory object */
	FILINFO fno;            /* File information object */
	uint32_t bw, br, i;

	static uint8_t s_val = 0;
	
	rc = f_mount(0, &Fatfs);     /* Register volume work area (never fails) */
	if (rc ) debug_printf_m("mount err");
	
	debug_printf_m("\nOpen an existing file (message.txt).\n");
	rc = f_open(&Fil, "MESSAGE.TXT", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) {
		debug_printf_s("open err, rc=");
		debug_printf_h(rc);
		debug_printf_m("");
		return ;
	}

	debug_printf_m("\nType the file content.\n");
while(1){
	bw = 0;
	for (i=0; i<512; i++) {
		wBuff[i]=bw + 'a';
		if (++bw >= 26)
			bw = 0;
	}
	wBuff[1] = s_val++;
	debug_printf_m("will write");
	for (;;) {
		rc = f_write(&Fil, wBuff, 512, &bw);
		if (rc || bw) break;
	}
	if (rc) {
		debug_printf_m("write err");
//		return ;
	}

	debug_printf_m("will read");
	for (;;) {
	    rc = f_read(&Fil, rBuff, sizeof rBuff, &br);  /* Read a chunk of file */
	    if (rc || !br) break;           /* Error or end of file */
	    for (i = 0; i < br; i++)        /* Type the data */
	    {
		//debug_printf_h(rBuff[i]);
		//debug_printf_s(",");
	}
	}
	if (rc) {
		debug_printf_m("read err");
//		return ;
	}
}
	debug_printf_m("\nClose the file.\n");
	rc = f_close(&Fil);
#endif
}




