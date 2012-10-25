#ifndef __LW_SD_H__
#define __LW_SD_H__

#include <stdint.h>
#include <stdbool.h>

/* SD卡信息结构体定义 */    
/* the information structure variable of SD Card*/          
typedef struct SD_STRUCT
{   
	uint8_t card_type;
	uint32_t block_num;   /* 卡中块的数量 */
	uint32_t block_len;   /* 卡的块长度(单位:字节) */
	uint32_t erase_unit;  /* 一次可擦除的块个数 */
	      
	uint32_t timeout_read;    /* 读块超时时间(单位: 8 SPI clock) */
	uint32_t timeout_write;   /* 写块超时时间(单位: 8 SPI clock) */
	uint32_t timeout_erase;   /* 擦块超时时间(单位: 8 SPI clock) */    
}sd_struct;

extern sd_struct sd_info;

#define   CARDTYPE_SD       0x00    // 卡型为SD  卡
#define   CARDTYPE_MMC      0x01    // 卡型为MMC 卡


/* 错误码 error code */
#define   SD_NO_ERR     0x00    // 函数执行成功
#define   SD_ERR_NO_CARD    0x01    // 卡没有完全插入到卡座中
#define   SD_ERR_USER_PARAM     0x02    // 用户使用API函数时，入口参数有错误
#define   SD_ERR_CARD_PARAM 0x03    // 卡中参数有错误（与本模块不兼容）
#define   SD_ERR_VOL_NOTSUSP    0x04    // 卡不支持3.3V供电
#define   SD_ERR_OVER_CARDRANGE 0x05    // 操作超出卡容量范围
#define   SD_ERR_UNKNOWN_CARD   0x06    // 无法识别卡型
                
/* SD命令可能返回的错误码 */
#define   SD_ERR_CMD_RESPTYPE   0x10    // 命令类型错误
#define   SD_ERR_CMD_TIMEOUT    0x11    // 卡命令响应超时
#define   SD_ERR_CMD_RESP   0x12    // 卡命令响应错误
            
/* 数据流错误码 */
#define   SD_ERR_DATA_CRC16     0x20    // 数据流CRC16校验不通过
#define   SD_ERR_DATA_START_TOK 0x21    // 读单块或多块时，数据开始令牌不正确
#define   SD_ERR_DATA_RESP  0x22    // 写单块或多块时，卡数据响应令牌不正确

/* 等待错误码 */
#define   SD_ERR_TIMEOUT_WAIT   0x30    // 写或擦操作时，发生超时错误
#define   SD_ERR_TIMEOUT_READ   0x31    // 读操作超时错误
#define   SD_ERR_TIMEOUT_WRITE  0x32    // 写操作超时错误   
#define   SD_ERR_TIMEOUT_ERASE  0x33    // 擦除操作超时错误
#define   SD_ERR_TIMEOUT_WAITIDLE 0x34  // 初始化卡时，等待卡退出空闲状态超时错误
        
/* 写操作可能返回的错误码 */
#define   SD_ERR_WRITE_BLK  0x40    // 写块数据错误
#define   SD_ERR_WRITE_BLKNUMS  0x41    // 写多块时，想要写入的块与正确写入的块数不一致
#define   SD_ERR_WRITE_PROTECT  0x42    // 卡外壳的写保护开关打在写保护位置




/* 定义在初始化阶段,等待SD卡退出空闲状态的重试次数 */
/* Number of tries to wait for the card to go idle during initialization */
#define SD_IDLE_WAIT_MAX  1000

int32_t lw_sd_init(void);
int32_t lw_sd_active(void);
int32_t lw_sd_fatfs_init(void);
int32_t lw_sd_fatfs_deinit(void);
int32_t lw_get_frame2sd(void);

#endif

