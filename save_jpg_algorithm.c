#include "projects_conf.h"

#if defined (STM_CAR)

#if 0
#define TARGET_X86
#else
#endif

#if defined (TARGET_X86)
#include<sys/stat.h>
#include<unistd.h>
#include<dirent.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#else
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "ff.h"
#include "save_jpg_algorithm.h"
#include "lw_gprs.h"
#include "lw_vc0706.h"
#include "lw_stm32_power.h"
#include "ctrl_gps_cam.h"
#endif

#define SEC_LEVEL_FILE_NUM 4
#define SEC_LEVEL_DIR_NUM 4
#define FIR_LEVEL_DIR_NUM 4
#define PATH_MAX_LEN 16

struct jpg_absolute_path {
    int8_t first[PATH_MAX_LEN];
    int8_t second[PATH_MAX_LEN];
    int8_t jpg[PATH_MAX_LEN];
};

#define JPG_HEADER_SIZE 128 /*预留128bytes*/
struct jpg_ext_msg_ {
	bool is_indep;
	float lat;
	float lon;
	float speed;
	float track;
};
struct jpg_ext_msg_ jpg_ext_msg;

struct memory_most_ {
	uint32_t gprs_jpg_index;
	uint32_t save_jpg_tail;
};
struct memory_most_ memory_most;
#define MEMORY_MOST_NAME "/m_m"

static bool exist_memory_most_file(void)
{
	#if defined (TARGET_X86)
	return true;
	#else
	int8_t ret;
	uint32_t bw;
	FIL fil;
	FILINFO finfo;
	struct memory_most_ memory_most;
	
	if ((ret = f_stat(MEMORY_MOST_NAME, &finfo)) != FR_OK) {
		if (ret != FR_NO_FILE)
			return false;
		memset(&memory_most, 0, sizeof(struct memory_most_));
		if (f_open(&fil, MEMORY_MOST_NAME, FA_CREATE_ALWAYS | FA_WRITE))
			return false;
		if (f_write(&fil, (const void *) (&memory_most), sizeof(struct memory_most_), &bw) ||
			bw < sizeof(struct memory_most_)) {
			f_close(&fil);
			return false;
		}
		if (f_close(&fil))
			return false;
	}
	return true;
	#endif
}

/*开机时必须调用*/
bool get_memory_most_from_sd(void)
{
	#if defined (TARGET_X86)
	return true;
	#else
	uint32_t br;
	FILINFO finfo;
	FIL fil;

	if (!exist_memory_most_file()) {
		goto err1;
	}
	if (f_open(&fil, MEMORY_MOST_NAME, FA_READ))
		goto err1;
	if (f_read(&fil, (void *) &memory_most, sizeof(struct memory_most_), &br) || 
			br < sizeof(struct memory_most_)) {
		goto err2;
	}
	f_close(&fil);
	return true;

err2:
	f_close(&fil);
err1:
	memset((void *)&memory_most, 0, sizeof(struct memory_most_));
	return false;
	#endif
}

bool save_momory_most_to_sd(void)
{
	#if defined (TARGET_X86)
	return true;
	#else
	uint32_t bw;
	FILINFO finfo;
	FIL fil;
	static uint32_t gprs_index_old=0;
	static uint32_t save_index_old=0;

	/*debug_printf_s("gprs_jpg_index:");
	debug_printf_h(memory_most.gprs_jpg_index);
	debug_printf_m("");
	debug_printf_s("save_jpg_tail:");
	debug_printf_h(memory_most.save_jpg_tail);
	debug_printf_m("");*/
	if(gprs_index_old == memory_most.gprs_jpg_index && save_index_old == memory_most.save_jpg_tail)
		return false;
	gprs_index_old = memory_most.gprs_jpg_index;
	save_index_old = memory_most.save_jpg_tail;
	
	if (f_open(&fil, MEMORY_MOST_NAME, FA_CREATE_ALWAYS | FA_WRITE))
		return false;
	if (f_write(&fil, (void *) &memory_most, sizeof(struct memory_most_), &bw) ||
		bw < sizeof(struct memory_most_)) {
		f_close(&fil);
		return false;
	}
	if (f_close(&fil))
		return false;
	return true;
	#endif
}

static bool exist_working_file(const int8_t * const wh, int8_t * const thename)
{
	#if defined (TARGET_X86)
	bool flag = false;
	DIR * dir;
	struct dirent *p;

	if ((dir =opendir(wh)) != NULL) {
		while ((p = readdir(dir)) != NULL) {
			if (p->d_name[strlen(p->d_name)-1] == '_') {
				strcpy(thename, p->d_name);
				flag = true;
				break;
			}
		}
	}
	closedir(dir);
	return flag;
	#else
	bool flag = false;
	DIR dir;
	FILINFO finfo;

	if (!f_opendir(&dir, wh)) {
		while (!f_readdir(&dir, &finfo) && finfo.fname[0] !=0) {
			if (finfo.fname[strlen(finfo.fname)-1] == '_') {
				strcpy(thename, finfo.fname);
				flag = true;
				break;
			}
		}
	}
	return flag;
	#endif
}

static int8_t get_save_path(struct jpg_absolute_path * const path)
{
	#if defined (TARGET_X86)
	bool ret;
	int8_t buf[PATH_MAX_LEN*3];
	int8_t name[PATH_MAX_LEN];

	ret = exist_working_file("./", name);
	if(!ret)
		return 1;
	sprintf(buf, "%s", "./");
	strcat(buf, name);
	strcpy(path->first, name);
	ret = exist_working_file(buf, name);
	if(!ret)
		return 1;
	strcat(buf, "/");
	strcat(buf, name);
	strcpy(path->second, name);
	ret = exist_working_file(buf, name);
	if(!ret)
		return 1;
	strcpy(path->jpg, name);
	return 0;
	#else
	bool ret;
	int8_t buf[PATH_MAX_LEN*3];
	int8_t name[PATH_MAX_LEN];

	ret = exist_working_file("/", name);
	if(!ret)
		return 1;
	sprintf(buf, "%s", "/");
	strcat(buf, name);
	strcpy(path->first, name);
	ret = exist_working_file(buf, name);
	if(!ret)
		return 1;
	strcat(buf, "/");
	strcat(buf, name);
	strcpy(path->second, name);
	ret = exist_working_file(buf, name);
	if(!ret)
		return 1;
	strcpy(path->jpg, name);
	return 0;
	#endif
}

/*
返回值：
0：同二级目录
1：同一级目录
2：不同的一级目录
*/
static int8_t nextfile_will_in_where(const struct jpg_absolute_path * const path)
{
	int32_t f,s,j;
	int8_t buf[PATH_MAX_LEN];

	strcpy(buf, path->first);
	buf[strlen(buf) -1] = '\0';
	f = atoi(buf);
	strcpy(buf, path->second);
	buf[strlen(buf) -1] = '\0';
	s = atoi(buf);
	strcpy(buf, path->jpg);
	buf[strlen(buf) -1] = '\0';
	j = atoi(buf);

	j += 1;
	if (j >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM) { /*重新*/
		return 2;
	}
	else if (j/(SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM)*(SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM) != f) {
		return 2;
	}
	else if (j/SEC_LEVEL_FILE_NUM*SEC_LEVEL_FILE_NUM != s) {
		return 1;
	}
	return 0;
}

static bool exist_pre_file(const struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t jpg[PATH_MAX_LEN];
	int8_t buf[PATH_MAX_LEN*3];

	#if defined (TARGET_X86)
	struct stat st;
	sprintf(buf, "%s", "./");
	strcat(buf, path->first);
	strcat(buf, "/");
	strcat(buf, path->second);
	strcat(buf, "/");
	strcpy(jpg, path->jpg);
	jpg[strlen(jpg) -1] = '\0';
	i = atoi(jpg);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(jpg, "%d", i);
	strcat(buf, jpg);
	if (stat(buf, &st))
		return false;
	return true;
	#else
	FILINFO finfo;
	sprintf(buf, "%s", "/");
	strcat(buf, path->first);
	strcat(buf, "/");
	strcat(buf, path->second);
	strcat(buf, "/");
	strcpy(jpg, path->jpg);
	jpg[strlen(jpg) -1] = '\0';
	i = atoi(jpg);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(jpg, "%d", i);
	strcat(buf, jpg);
	if (f_stat(buf, &finfo))
		return false;
	return true;
	#endif
}

static bool change_to_next_file(const struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t jpg[PATH_MAX_LEN];
	int8_t buf1[PATH_MAX_LEN*3];
	int8_t buf2[PATH_MAX_LEN*3];

	#if defined (TARGET_X86)
	int8_t buf3[PATH_MAX_LEN*6];
	sprintf(buf1, "%s", "./");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcat(buf1, path->second);
	strcat(buf1, "/");
	strcat(buf1, path->jpg);
	strcpy(buf2, buf1);
	buf2[strlen(buf2) -1] = '\0';
	sprintf(buf3, "mv %s %s", buf1, buf2);
	system(buf3); /*改变原来文件名*/
	return true;
	#else
	sprintf(buf1, "%s", "/");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcat(buf1, path->second);
	strcat(buf1, "/");
	strcat(buf1, path->jpg);
	strcpy(buf2, buf1);
	buf2[strlen(buf2) -1] = '\0';
	if (f_rename(buf1, buf2))
		return false;

	/*改变可以发送的jpg指针*/
	strcpy(buf1, path->jpg);
	buf1[strlen(buf1) -1] = '\0';
	i = atoi(buf1);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	memory_most.save_jpg_tail = i;	/*指向下一个将要录像的文件*/

	return true;
	#endif
}

static bool change_pre_file(const struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t jpg[PATH_MAX_LEN];
	int8_t buf1[PATH_MAX_LEN*3];
	int8_t buf2[PATH_MAX_LEN*3];

	#if defined (TARGET_X86)
	int8_t buf3[PATH_MAX_LEN*6];
	sprintf(buf1, "%s", "./");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcat(buf1, path->second);
	strcat(buf1, "/");
	strcpy(buf2, buf1);
	strcpy(jpg, path->jpg);
	jpg[strlen(jpg) -1] = '\0';
	i = atoi(jpg);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(jpg, "%d", i);
	strcat(buf1, jpg);
	strcat(jpg, "_");
	strcat(buf2, jpg);
	sprintf(buf3, "mv %s %s", buf1, buf2);
	system(buf3); /*改变原来文件名*/
	return true;
	#else
	sprintf(buf1, "%s", "/");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcat(buf1, path->second);
	strcat(buf1, "/");
	strcpy(buf2, buf1);
	strcpy(jpg, path->jpg);
	jpg[strlen(jpg) -1] = '\0';
	i = atoi(jpg);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(jpg, "%d", i);
	strcat(buf1, jpg);
	strcat(jpg, "_");
	strcat(buf2, jpg);
	if (f_rename(buf1, buf2))
		return false;
	return true;
	#endif
}

static bool create_file(struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t jpg[PATH_MAX_LEN];
	int8_t buf1[PATH_MAX_LEN*3];
	int8_t buf2[PATH_MAX_LEN*3];

	#if defined (TARGET_X86)
	sprintf(buf1, "%s", "./");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcat(buf1, path->second);
	strcat(buf1, "/");
	strcpy(jpg, path->jpg);
	jpg[strlen(jpg) -1] = '\0';
	i = atoi(jpg);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(jpg, "%d", i);
	strcat(jpg, "_");
	strcat(buf1, jpg);
	sprintf(buf2, "touch %s", buf1);
	system(buf2);

	strcpy(path->jpg, jpg);
	return true;
	#else
	FIL fil;
	sprintf(buf1, "%s", "/");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcat(buf1, path->second);
	strcat(buf1, "/");
	strcpy(jpg, path->jpg);
	jpg[strlen(jpg) -1] = '\0';
	i = atoi(jpg);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(jpg, "%d", i);
	strcat(jpg, "_");
	strcat(buf1, jpg);

	if (f_open(&fil, buf1, FA_CREATE_ALWAYS))
		return false;
	if (f_close(&fil))
		return false;

	strcpy(path->jpg, jpg);
	return true;
	#endif
}

static bool change_to_next_sec_path(struct jpg_absolute_path * const path)
{
	int8_t s[PATH_MAX_LEN];
	int8_t buf1[PATH_MAX_LEN*2];
	int8_t buf2[PATH_MAX_LEN*2];

	#if defined (TARGET_X86)
	int8_t buf3[PATH_MAX_LEN*2];
	sprintf(buf1, "%s", "./");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcpy(buf2, buf1);
	strcpy(s, path->second);
	strcat(buf1, s);
	s[strlen(s) -1] = '\0';
	strcat(buf2, s);
	sprintf(buf3, "mv %s %s", buf1, buf2);
	system(buf3);
	strcpy(path->second, s);
	return true;
	#else
	sprintf(buf1, "%s", "/");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcpy(buf2, buf1);
	strcpy(s, path->second);
	strcat(buf1, s);
	s[strlen(s) -1] = '\0';
	strcat(buf2, s);
	if (f_rename(buf1, buf2))
		return false;
	strcpy(path->second, s);
	return true;
	#endif
}

static bool exist_pre_sec_path(const struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t s[PATH_MAX_LEN];
	int8_t buf[PATH_MAX_LEN*2];

	#if defined (TARGET_X86)
	struct stat st;
	sprintf(buf, "%s", "./");
	strcat(buf, path->first);
	strcat(buf, "/");
	strcpy(s, path->jpg);
	s[strlen(s) -1] = '\0';
	i = atoi(s);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(s, "%d", i);
	strcat(buf, s);
	if (stat(buf, &st))
		return false;
	return true;
	#else
	FILINFO finfo;
	sprintf(buf, "%s", "/");
	strcat(buf, path->first);
	strcat(buf, "/");
	strcpy(s, path->jpg);
	s[strlen(s) -1] = '\0';
	i = atoi(s);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(s, "%d", i);
	strcat(buf, s);
	if (f_stat(buf, &finfo))
		return false;
	return true;
	#endif
}

static bool change_pre_sec_path(struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t s[PATH_MAX_LEN];
	int8_t buf1[PATH_MAX_LEN*2];
	int8_t buf2[PATH_MAX_LEN*2];

	#if defined (TARGET_X86)
	int8_t buf3[PATH_MAX_LEN*2];
	sprintf(buf1, "%s", "./");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcpy(buf2, buf1);
	strcpy(s, path->jpg);
	s[strlen(s) -1] = '\0';
	i = atoi(s);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(s, "%d", i);
	strcat(buf1, s);
	strcat(s, "_");
	strcat(buf2, s); 
	sprintf(buf3, "mv %s %s", buf1, buf2);
	system(buf3);
	
	strcpy(path->second, s);
	return true;
	#else
	sprintf(buf1, "%s", "/");
	strcat(buf1, path->first);
	strcat(buf1, "/");
	strcpy(buf2, buf1);
	strcpy(s, path->jpg);
	s[strlen(s) -1] = '\0';
	i = atoi(s);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(s, "%d", i);
	strcat(buf1, s);
	strcat(s, "_");
	strcat(buf2, s); 
	if (f_rename(buf1, buf2))
		return false;

	strcpy(path->second, s);
	return true;
	#endif
}

static bool create_sec_path(struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t s[PATH_MAX_LEN];
	int8_t buf[PATH_MAX_LEN*2];

	#if defined (TARGET_X86)
	int8_t buf2[PATH_MAX_LEN*2];
	sprintf(buf, "%s", "./");
	strcat(buf, path->first);
	strcat(buf, "/");
	strcpy(s, path->jpg);
	s[strlen(s) -1] = '\0';
	i = atoi(s);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(s, "%d", i);
	strcat(s, "_");
	strcat(buf, s);
	sprintf(buf2, "mkdir -p %s", buf);
	system(buf2);
	
	strcpy(path->second, s);
	return true;
	#else
	sprintf(buf, "%s", "/");
	strcat(buf, path->first);
	strcat(buf, "/");
	strcpy(s, path->jpg);
	s[strlen(s) -1] = '\0';
	i = atoi(s);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(s, "%d", i);
	strcat(s, "_");
	strcat(buf, s);
	if (f_mkdir(buf))
		return false;

	strcpy(path->second, s);
	return true;
	#endif
}

static bool change_to_next_fir_path(struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t f[PATH_MAX_LEN];
	int8_t buf1[PATH_MAX_LEN*2];
	int8_t buf2[PATH_MAX_LEN*2];
	int8_t buf3[PATH_MAX_LEN*2];

	#if defined (TARGET_X86)
	sprintf(buf1, "%s", "./");
	strcpy(buf2, buf1);
	strcpy(f, path->first);
	strcat(buf1, f);
	f[strlen(f) -1] = '\0';
	strcat(buf2, f);
	sprintf(buf3, "mv %s %s", buf1, buf2);
	system(buf3);
	strcpy(path->first, f);
	return true;
	#else
	sprintf(buf1, "%s", "/");
	strcpy(buf2, buf1);
	strcpy(f, path->first);
	strcat(buf1, f);
	f[strlen(f) -1] = '\0';
	strcat(buf2, f);
	if (f_rename(buf1, buf2))
		return false;
	strcpy(path->first, f);
	return true;
	#endif
}

static bool exist_pre_fir_path(const struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t f[PATH_MAX_LEN];
	int8_t buf[PATH_MAX_LEN*2];

	#if defined (TARGET_X86)
	struct stat st;
	sprintf(buf, "%s", "./");
	strcpy(f, path->jpg);
	f[strlen(f) -1] = '\0';
	i = atoi(f);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(f, "%d", i);
	strcat(buf, f);
	if (stat(buf, &st))
		return false;
	return true;
	#else
	FILINFO finfo;
	sprintf(buf, "%s", "/");
	strcpy(f, path->jpg);
	f[strlen(f) -1] = '\0';
	i = atoi(f);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(f, "%d", i);
	strcat(buf, f);
	if (f_stat(buf, &finfo))
		return false;
	return true;
	#endif
}

static bool change_pre_fir_path(struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t f[PATH_MAX_LEN];
	int8_t buf1[PATH_MAX_LEN*2];
	int8_t buf2[PATH_MAX_LEN*2];

	#if defined (TARGET_X86)
	int8_t buf3[PATH_MAX_LEN*2];
	sprintf(buf1, "%s", "./");
	strcpy(buf2, buf1);
	strcpy(f,path->jpg);
	f[strlen(f) -1] = '\0';
	i = atoi(f);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(f, "%d", i);
	strcat(buf1, f);
	strcat(f, "_");
	strcat(buf2, f);
	sprintf(buf3, "mv %s %s", buf1, buf2);
	system(buf3);
	
	strcpy(path->first, f);
	return true;
	#else
	sprintf(buf1, "%s", "/");
	strcpy(buf2, buf1);
	strcpy(f,path->jpg);
	f[strlen(f) -1] = '\0';
	i = atoi(f);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(f, "%d", i);
	strcat(buf1, f);
	strcat(f, "_");
	strcat(buf2, f);
	if (f_rename(buf1, buf2))
		return false;
	
	strcpy(path->first, f);
	return true;
	#endif
}

static bool create_fir_path(struct jpg_absolute_path * const path)
{
	int32_t i;
	int8_t f[PATH_MAX_LEN];
	int8_t buf[PATH_MAX_LEN*2];

	#if defined (TARGET_X86)
	int8_t buf2[PATH_MAX_LEN*2];
	sprintf(buf, "%s", "./");
	strcpy(f, path->jpg);
	f[strlen(f) -1] = '\0';
	i = atoi(f);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(f, "%d", i);
	strcat(f, "_");
	strcat(buf, f);
	sprintf(buf2, "mkdir -p %s", buf);
	system(buf2);
	
	strcpy(path->first, f);
	return true;
	#else
	sprintf(buf, "%s", "/");
	strcpy(f, path->jpg);
	f[strlen(f) -1] = '\0';
	i = atoi(f);
	i++;
	if (i >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		i = 0;
	sprintf(f, "%d", i);
	strcat(f, "_");
	strcat(buf, f);
	if (f_mkdir(buf))
		return false;
	
	strcpy(path->first, f);
	return true;
	#endif
}

static bool create_mul_path_file(struct jpg_absolute_path * const path)
{
	int8_t ret;
	struct jpg_absolute_path lpath;

	ret = nextfile_will_in_where(path);
	if(0 == ret) { /*同二级目录*/
		if (!change_to_next_file(path))
			return false;
		if (exist_pre_file(path)) {
			if (!change_pre_file(path))
				return false;
			return true;
		}
		else {
			if (!create_file(path))
				return false;
			return true;
		}
		
		/*if (exist_pre_file(path)) {
			return change_file_and_clear(path);
		}
		return create_file(path);*/
	}
	else if (1 == ret) { /*同一级目录*/
		lpath = *path;
		if (!change_to_next_sec_path(&lpath))/*改变原二级目录的名称不带_*/
			return false;
		if (!change_to_next_file(&lpath))
			return false;

		if (exist_pre_sec_path(path)) {
			if (!change_pre_sec_path(path)) {
				return false;
			}
		}
		else {
			if (!create_sec_path(path))
				return false;
		}
		if (exist_pre_file(path)) {
			if (!change_pre_file(path))
				return false;
			return true;
		}
		else {
			if (!create_file(path))
				return false;
			return true;
		}
	}
	else if (2 == ret) { /*不同的一级目录*/
		lpath = *path;
		if (!change_to_next_fir_path(&lpath))
			return false;
		if (!change_to_next_sec_path(&lpath))/*改变原二级目录的名称不带_*/
			return false;
		if (!change_to_next_file(&lpath))
			return false;

		if (exist_pre_fir_path(path)) {
			if (!change_pre_fir_path(path)) {
				return false;
			}
		}
		else {
			if (!create_fir_path(path))
				return false;
		}
		if (exist_pre_sec_path(path)) {
			if (!change_pre_sec_path(path)) {
				return false;
			}
		}
		else {
			if (!create_sec_path(path))
				return false;
		}
		if (exist_pre_file(path)) {
			if (!change_pre_file(path))
				return false;
			return true;
		}
		else {
			if (!create_file(path))
				return false;
			return true;
		}

	}
	else { /*未定义*/

	}
	return false;
}

static bool create_orig_path_file(void)
{
	#if defined (TARGET_X86)
	//system("mkdir -p ./0_/0_");
	//system("touch ./0_/0_/0_");
	int32_t i;
	struct jpg_absolute_path path;
	int8_t name[PATH_MAX_LEN];
	if (!exist_working_file("./", name)) {
		strcpy(path.first, "0_");
		strcpy(path.second, "0_");
		sprintf(path.jpg, "%d", ((int32_t)0) -1);
		strcat(path.jpg, "_");
		if (exist_pre_fir_path(&path)) {
			if (!change_pre_fir_path(&path)) {
				return false;
			}
		}
		else {
			if (!create_fir_path(&path))
				return false;
		}
		if (exist_pre_sec_path(&path)) {
			if (!change_pre_sec_path(&path)) {
				return false;
			}
		}
		else {
			if (!create_sec_path(&path))
				return false;
		}
		if (exist_pre_file(&path)) {
			if (!change_pre_file(&path))
				return false;
			return true;
		}
		else {
			if (!create_file(&path))
				return false;
			return true;
		}
	}
	strcpy(path.first, name);
	strcpy(name, "./");
	strcat(name, path.first);
	if (!exist_working_file(name, name)) {
		strcpy(path.second, path.first);
		strcpy(name, path.first);
		name[strlen(name) -1] = '\0';
		i = atoi(name);
		i--;
		sprintf(path.jpg, "%d", i);
		strcat(path.jpg, "_");
		if (exist_pre_sec_path(&path)) {
			if (!change_pre_sec_path(&path)) {
				return false;
			}
		}
		else {
			if (!create_sec_path(&path))
				return false;
		}
		if (exist_pre_file(&path)) {
			if (!change_pre_file(&path))
				return false;
			return true;
		}
		else {
			if (!create_file(&path))
				return false;
			return true;
		}
	}
	strcpy(path.second, name);
	name[strlen(name) -1] = '\0';
	i = atoi(name);
	i--;
	sprintf(path.jpg, "%d", i);
	strcat(path.jpg, "_");
	if (exist_pre_file(&path)) {
		if (!change_pre_file(&path))
			return false;
		return true;
	}
	else {
		if (!create_file(&path))
			return false;
		return true;
	}
	#else
	int32_t i;
	struct jpg_absolute_path path;
	int8_t name[PATH_MAX_LEN];
	if (!exist_working_file("/", name)) {
		strcpy(path.first, "0_");
		strcpy(path.second, "0_");
		sprintf(path.jpg, "%d", ((int32_t)0) -1);
		strcat(path.jpg, "_");
		if (exist_pre_fir_path(&path)) {
			if (!change_pre_fir_path(&path)) {
				return false;
			}
		}
		else {
			if (!create_fir_path(&path))
				return false;
		}
		if (exist_pre_sec_path(&path)) {
			if (!change_pre_sec_path(&path)) {
				return false;
			}
		}
		else {
			if (!create_sec_path(&path))
				return false;
		}
		if (exist_pre_file(&path)) {
			if (!change_pre_file(&path))
				return false;
			return true;
		}
		else {
			if (!create_file(&path))
				return false;
			return true;
		}
	}
	strcpy(path.first, name);
	strcpy(name, "/");
	strcat(name, path.first);
	if (!exist_working_file(name, name)) {
		strcpy(path.second, path.first);
		strcpy(name, path.first);
		name[strlen(name) -1] = '\0';
		i = atoi(name);
		i--;
		sprintf(path.jpg, "%d", i);
		strcat(path.jpg, "_");
		if (exist_pre_sec_path(&path)) {
			if (!change_pre_sec_path(&path)) {
				return false;
			}
		}
		else {
			if (!create_sec_path(&path))
				return false;
		}
		if (exist_pre_file(&path)) {
			if (!change_pre_file(&path))
				return false;
			return true;
		}
		else {
			if (!create_file(&path))
				return false;
			return true;
		}
	}
	strcpy(path.second, name);
	name[strlen(name) -1] = '\0';
	i = atoi(name);
	i--;
	sprintf(path.jpg, "%d", i);
	strcat(path.jpg, "_");
	if (exist_pre_file(&path)) {
		if (!change_pre_file(&path))
			return false;
		return true;
	}
	else {
		if (!create_file(&path))
			return false;
		return true;
	}
	#endif
}

static bool create_nextfile(struct jpg_absolute_path * const path)
{
	int32_t i;

	if (NULL == path) {
		return create_orig_path_file();
	}
	/*path->jpg[strlen(path->jpg) -1] = '\0';
	i = atoi(path->jpg);
	sprintf(path->jpg, "%d", i+1);
	strcat(path->jpg, "_");*/
	return create_mul_path_file(path);
}

void save_jpg_ext_msg_gps(float lat, float lon, float speed, float track)
{
	jpg_ext_msg.lat = lat;
	jpg_ext_msg.lon = lon;
	jpg_ext_msg.speed = speed;
	jpg_ext_msg.track = track;
}

static bool save_data_file(const struct jpg_absolute_path * const path)
{
	int8_t buf[PATH_MAX_LEN*3];
	
	#if defined (TARGET_X86)
	uint32_t wlen;
	FILE * fp;
	strcpy(buf, "./");
	strcat(buf, path->first);
	strcat(buf, "/");
	strcat(buf, path->second);
	strcat(buf, "/");
	strcat(buf, path->jpg);
	fp = fopen(buf, "w");
	if (!fp)
		return false;
	wlen = fwrite(wbuf, 1, len, fp);
	if (wlen != len) {
		fclose(fp);
		return false;
	}
	fclose(fp);
	return true;
	#else
	bool flag = false;
	bool ret;
	uint32_t wlen, bw;
	FIL fil;
	uint8_t *p;
	uint32_t len;
	uint8_t buf2[JPG_HEADER_SIZE];
	
	strcpy(buf, "/");
	strcat(buf, path->first);
	strcat(buf, "/");
	strcat(buf, path->second);
	strcat(buf, "/");
	strcat(buf, path->jpg);
	if (f_open(&fil, buf, FA_CREATE_ALWAYS | FA_WRITE))
		goto err1;
	memset(buf2, 0, JPG_HEADER_SIZE);
	memcpy(buf2, (void *)&jpg_ext_msg, sizeof(struct jpg_ext_msg_));
	if (f_write(&fil, buf2, JPG_HEADER_SIZE, &bw) || bw < JPG_HEADER_SIZE) {
		goto err2;
	}
	#if 1
	if (!cam_ready_used_save_sd())
		goto err2;
	do {
		len = 0;
		ret = get_cam_used_save_sd(&p, &len);
		if (len)
		if (f_write(&fil, p, len, &bw) || bw < len) {
			goto err2;
		}
		if (!len && !ret)
			goto err2;
	} while(!ret);
	#endif
	flag = true;
err2:
	f_close(&fil);
err1:
	return flag;
	#endif
}

//bool save_jpg(const uint8_t *buf, const uint32_t len)
static bool save_jpg(void)
{
	int8_t ret;
	struct jpg_absolute_path path;

rp:
	ret = get_save_path(&path);
	if (-1 == ret)
		return false;
	else if (1 == ret) {
		if (!create_nextfile(NULL))
			return false;
		goto rp;
	}
	//if (!save_data_file(&path, buf, len))
	if (!save_data_file(&path))
		return false;
	if (!create_nextfile(&path))
		return false;
	return true;
}

void save_jpg_algorithm(void)
{
	if (will_save_memory_most()) {
		save_momory_most_to_sd();
	}
	if (gprs_offline_20min() && (is_vc0706_unlock() || is_vc0706_lock_by_sd())) {
		if (is_send_cam()) {
			set_no_send_cam();
			set_vc0706_lock_by_sd();
			save_jpg();
			set_vc0706_unlock();
		}
	}
}


#if 1
enum {
	STATUE_SEND_JPG_GPS = 0,
	STATUE_SEND_JPG_L_CMD,
	STATUE_SEND_JPG,
	STATUE_WAIT_L_CMD,
	STATUE_FINISH,
};
static int8_t send_jpg_from_sd_status;
static FIL sfil;
void send_jpg_from_sd_global_init(void)
{
	memset(&sfil, 0, sizeof(FIL));
}

static bool calc_jpg_path(int8_t *path, uint32_t jpgname)
{
	uint32_t f;
	uint32_t s;
	FILINFO finfo;
	int8_t buf[PATH_MAX_LEN*3];

	f = jpgname/(SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM) * 
		(SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM);
	s = jpgname/SEC_LEVEL_FILE_NUM*SEC_LEVEL_FILE_NUM;
	strcpy(path, "/");
	sprintf(buf, "%d", f);
	strcat(path, buf);
	if (f_stat(path, &finfo)) {
		strcat(path, "_");
	}
	strcat(path, "/");
	sprintf(buf, "%d", s);
	strcat(path, buf);
	if (f_stat(path, &finfo)) {
		strcat(path, "_");
	}
	strcat(path, "/");
	sprintf(buf, "%d", jpgname);
	strcat(path, buf);
	if (f_stat(path, &finfo)) {
		return false;
	}
	return true;
}

static void change_memory_most(void)
{
	if (memory_most.gprs_jpg_index != memory_most.save_jpg_tail)
		memory_most.gprs_jpg_index++;
	if (memory_most.gprs_jpg_index >= SEC_LEVEL_FILE_NUM*SEC_LEVEL_DIR_NUM*FIR_LEVEL_DIR_NUM)
		memory_most.gprs_jpg_index = 0;
}

bool have_jpg_from_sd_to_send(void)
{
	/*检查是否有录像可以发*/
	if (memory_most.gprs_jpg_index == memory_most.save_jpg_tail)
		return false;
	else
		return true;
}

bool send_jpg_from_sd_init(void)
{
	int8_t path[PATH_MAX_LEN*3];

	/*检查是否有录像可以发*/
	if(!have_jpg_from_sd_to_send())
		return false;
	
	if (sfil.fs != 0) {
		f_close(&sfil);
		memset(&sfil, 0, sizeof(FIL));
	}
	if (!calc_jpg_path(path, memory_most.gprs_jpg_index)) {
		change_memory_most();
		return false;
	}
	if (f_open(&sfil, (const TCHAR *)path, FA_READ)) {
		return false;
	}
	send_jpg_from_sd_status = STATUE_SEND_JPG_GPS;
	return true;
}

/*
返回值:
1:	发送jpg完毕
0:	无错误
-1:	发送jpg过程中出错,因为只要发了jpg的长度给服务器，服务器会认为接下来的数据都是jpg
-2:	发送jpg前出错
*/
int8_t send_jpg_from_sd(void)
{
	extern volatile unsigned int wait_l_cmd_timeout;
	int8_t flag = 0;
	FRESULT res;
	uint8_t *cambuf;
	uint32_t cammlen;
	uint32_t br;
	uint32_t fsize;
	struct jpg_ext_msg_ jem;
	uint8_t buf[128];
	static uint32_t jpg_size;
	static uint32_t send_cnt;

	switch (send_jpg_from_sd_status) {
		case STATUE_SEND_JPG_GPS:
		#if 1
		get_vc0706_rbuf_info(&cambuf, &cammlen);
		if (f_lseek(&sfil, 0)) {
			flag = -2;
			send_jpg_from_sd_status = STATUE_FINISH;
			goto fnh;
		}
		if (f_read(&sfil, cambuf, JPG_HEADER_SIZE, &br) || br < JPG_HEADER_SIZE) {
			flag = -2;
			send_jpg_from_sd_status = STATUE_FINISH;
			goto fnh;
		}
		else {
			memcpy(&jem, cambuf, sizeof(struct jpg_ext_msg_));
			/*GPS发送处理*/
			send_jpg_from_sd_status = STATUE_SEND_JPG_L_CMD;
		}
		break;
		case STATUE_SEND_JPG_L_CMD:
		jpg_size = f_size(&sfil);
		if (jpg_size < JPG_HEADER_SIZE) {
			flag = -2;
			send_jpg_from_sd_status = STATUE_FINISH;
			goto fnh;
		}
		else {
			#define CAM_CMD_L "L:%X#"
			#if 1
			jpg_size -= JPG_HEADER_SIZE;
			#else
			if (f_lseek(&sfil, 0)) {
				flag = -2;
				send_jpg_from_sd_status = STATUE_FINISH;
				goto fnh;
			}
			#endif
			send_cnt = 0;
			send_jpg_from_sd_status = STATUE_SEND_JPG;
			/*send L cmd*/
			sprintf(buf, CAM_CMD_L, jpg_size);
			gprs_send_cmd_f(1, buf, strlen(buf));
		}
		break;
		case STATUE_SEND_JPG:
		get_vc0706_rbuf_info(&cambuf, &cammlen);
		res = f_read(&sfil, cambuf, 1024, &br);
		send_cnt += br;
		if ((FR_OK == res) || (send_cnt >= jpg_size)) {
			/*GPRS发送*/
			gprs_send_cmd_f(1, cambuf, 1024);
			wait_l_cmd_timeout = 3000;
			send_jpg_from_sd_status = STATUE_WAIT_L_CMD;
		}
		else {
			flag = -1;
			send_jpg_from_sd_status = STATUE_FINISH;
			goto fnh;
		}

		break;
		case STATUE_WAIT_L_CMD:
		/*wait for L cmd*/
		if (lw_get_seqed_msg("L:")) {
			if (send_cnt >= jpg_size) {
				flag = 1;
				send_jpg_from_sd_status = STATUE_FINISH;
				goto fnh;
			}
			else {
				send_jpg_from_sd_status = STATUE_SEND_JPG;
			}
		}
		else if (!wait_l_cmd_timeout){
			flag = -1;
			send_jpg_from_sd_status = STATUE_FINISH;
			goto fnh;
		}
		break;
		#endif
		case STATUE_FINISH:
fnh:
		f_close(&sfil);
		memset(&sfil, 0, sizeof(FIL));
		change_memory_most();
		send_jpg_from_sd_status = STATUE_SEND_JPG_GPS;
		break;
		default:
		break;
	}

	return flag;
}
#endif
/*
int main(void)
{
	bool ret;
	uint32_t cnt = 1;
	uint32_t len;
	uint8_t buf[16];

	while (1) {
		if (cnt==5000)
			break;
		sprintf(buf, "%d", cnt++);
		len = strlen(buf);

		ret = save_jpg(buf, len);
		if (!ret)
			return -1;
	}

	return 0;
}
*/
#endif

