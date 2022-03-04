/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: usr_time_sync.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Time synchronization MASTER functionality
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef __USR_TIME_SYNC_H__
#define __USR_TIME_SYNC_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "time_sync.h"

#define USR_TIME_SYNC_TIMESTAMP_TO_USEC     TIME_SYNC_TIMESTAMP_TO_USEC
#define USR_TIME_SYNC_MSEC_TO_TICK          TIME_SYNC_MSEC_TO_TICK

typedef struct{
    uint32_t ts_led_pin;
    uint32_t ts_meas_pin;
}usr_timesync_config_t;

void sync_timer_init();
void ts_imu_trigger_enable(void);
void ts_imu_trigger_disable(void);
bool ts_get_imu_trigger_enabled(void);
void ts_print_sync_time();
uint64_t usr_ts_timestamp_get_ticks_u64();

#endif