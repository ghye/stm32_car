
#include "diskio.h"

DSTATUS Stat = STA_NOINIT;

DSTATUS disk_initialize (
    BYTE drv        /* Physical drive number (0) */
)
{
	BYTE n, cmd, ty, ocr[4];  

	if (drv) return STA_NOINIT;         /* Supports only single drive */

	/* FIXME:*/
	//if (Stat & STA_NODISK) return Stat; /* No card in the socket */

	if (!lw_sd_disk_initialize()) {
		Stat = 0;
		return 0;
	}
	else
		return STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE drv        /* Physical drive number (0) */
)
{
	if(drv)
		return STA_NOINIT;
	
	return Stat;

#if 0
	if (drv || Stat) return STA_NOINIT;     /* Supports only single drive */

	if (!lw_sd_disk_status())
		return 0;
	else
		return STA_NOINIT;
	//return Stat;
#endif
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read (
    BYTE drv,           /* Physical drive number (0) */
    BYTE *buff,         /* Pointer to the data buffer to store read data */
    DWORD sector,       /* Start sector number (LBA) */
    BYTE count          /* Sector count (1..255) */
)
{
	if (drv || Stat || !count)
		return RES_ERROR;

	if (lw_sd_disk_read(buff, sector, count))
		return RES_ERROR;
	
	return RES_OK;
}


DRESULT disk_write (
    BYTE drv,           /* Physical drive number (0) */
    const BYTE *buff,   /* Pointer to the data to be written */
    DWORD sector,       /* Start sector number (LBA) */
    BYTE count          /* Sector count (1..255) */
)
{
	if(drv || Stat || !count)
		return RES_ERROR;

	if (lw_sd_disk_write(buff, sector, count))
		return RES_ERROR;

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl (
    BYTE drv,       /* Physical drive number (0) */
    BYTE ctrl,      /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
	DRESULT ret;
	BYTE *ptr;

	if(drv || Stat)
		return RES_ERROR;

	ptr = buff;
	ret = RES_ERROR;
	
	if (drv)
		return RES_PARERR;

	if (ctrl == CTRL_POWER) {
		switch (*ptr) {
		case 0:
			if (!lw_sd_disk_power_off())
				ret = RES_OK;
			break;
		case 1:
			if (!lw_sd_disk_power_on())
				ret = RES_OK;
			break;
		case 2:
			*(ptr + 1) = lw_sd_disk_power_sta();
			ret = RES_OK;
			break;
		default:
			ret = RES_PARERR;
			break;
		}
	}
	else {
		switch (ctrl) {
		case CTRL_SYNC:
			if (!lw_sd_disk_ctrl_sync())
				ret = RES_OK;
			break;
		case GET_SECTOR_COUNT:
			if (!lw_sd_disk_get_sector_count(buff))
				ret = RES_OK;
			break;
		case GET_SECTOR_SIZE:
			if (!lw_sd_disk_get_sector_size(buff))
				ret = RES_OK;			
			break;
		case GET_BLOCK_SIZE:
			if (!lw_sd_disk_get_blokc_size(buff))
				ret = RES_OK;			
			break;
		case MMC_GET_TYPE:
			if (!lw_sd_disk_get_type(buff))
				ret = RES_OK;			
			break;
		case MMC_GET_CSD:
			if (!lw_sd_disk_get_csd(buff))
				ret = RES_OK;			
			break;
		case MMC_GET_CID:
			if (!lw_sd_disk_get_cid(buff))
				ret = RES_OK;			
			break;
		case MMC_GET_OCR:
			if (!lw_sd_disk_get_ocr(buff))
				ret = RES_OK;			
			break;
		case MMC_GET_SDSTAT:
			if (!lw_sd_disk_get_sdstat(buff))
				ret = RES_OK;			
			break;
		default:
			ret = RES_PARERR;
			break;
		}
	}

	return ret;
}


DWORD get_fattime (void)
{
	return lw_sd_disk_get_fattime();
}
