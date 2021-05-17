#ifndef __USR_TIME_SYNC_H__
#define __USR_TIME_SYNC_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "time_sync.h"


void ts_imu_trigger_enable(void);
bool ts_get_imu_trigger_enabled(void);
void sync_timer_init(ts_evt_handler_t ts_evt_callback);



#endif