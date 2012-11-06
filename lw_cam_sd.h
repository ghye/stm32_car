#ifndef __LW_CAM_SD_H__
#define __LW_CAM_SD_H__


enum cam2sdstate{
	CAM_TO_SD_ERR = 0,
	CAM_TO_SD_FINISH,
	CAM_TO_SD_CONTINUE,
};

enum camfromsdstate {
	CAM_FROM_SD_ERR = 0,
	CAM_FROM_SD_OK,
	CAM_FROM_SD_NODATA,
};

enum cam2sdstate lw_push_cam_to_sd(void);

/*Ã¿´Î¶Á1024×Ö½Ú*/
enum camfromsdstate lw_get_cam_from_sd(uint8_t **stream, uint32_t *streamlen);

#endif

