#ifndef __SAVE_JPG_ALGORITHM_H__
#define __SAVE_JPG_ALGORITHM_H__

#include <stdint.h>
#include <stdbool.h>

bool get_memory_most_from_sd(void);
bool save_momory_most_to_sd(void);
void save_jpg_ext_msg_gps(float lat, float lon, float speed, float track);
void save_jpg_algorithm(void);
void send_jpg_from_sd_global_init(void);
bool send_jpg_from_sd_init(void);
int8_t send_jpg_from_sd(void);
bool have_jpg_from_sd_to_send(void);

#endif

