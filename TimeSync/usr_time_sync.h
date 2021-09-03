#ifndef __USR_TIME_SYNC_H__
#define __USR_TIME_SYNC_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "time_sync.h"


typedef struct{
    uint32_t ts_led_pin;
    uint32_t ts_meas_pin;
}usr_timesync_config_t;

void sync_timer_init();

void ts_imu_trigger_enable(void);
void ts_imu_trigger_disable(void);
bool ts_get_imu_trigger_enabled(void);
void ts_print_sync_time();


#endif