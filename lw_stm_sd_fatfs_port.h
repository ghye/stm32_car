#ifndef __LW_STM_SD_FATFS_PORT_H__
#define __LW_STM_SD_FATFS_PORT_H__

#include "integer.h"
#include "ff.h"

DSTATUS disk_initialize (
    BYTE drv        /* Physical drive number (0) */
);


/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (
    BYTE drv        /* Physical drive number (0) */
);

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read (
    BYTE drv,           /* Physical drive number (0) */
    BYTE *buff,         /* Pointer to the data buffer to store read data */
    DWORD sector,       /* Start sector number (LBA) */
    BYTE count          /* Sector count (1..255) */
);


DRESULT disk_write (
    BYTE drv,           /* Physical drive number (0) */
    const BYTE *buff,   /* Pointer to the data to be written */
    DWORD sector,       /* Start sector number (LBA) */
    BYTE count          /* Sector count (1..255) */
);


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl (
    BYTE drv,       /* Physical drive number (0) */
    BYTE ctrl,      /* Control code */
    void *buff      /* Buffer to send/receive control data */
);

DWORD get_fattime (void);


#endif
