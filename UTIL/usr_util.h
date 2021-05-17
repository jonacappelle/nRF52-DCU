#ifndef _USR_UTIL_H__
#define _USR_UTIL_H__


#define PIN_CPU_ACTIVITY    19

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app_timer.h"
#include "app_fifo.h"


void usr_gpio_init();

void check_cpu_activity();

uint32_t calculate_string_len(char * string);


// APP Scheduler
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 200                                      /**< Maximum number of events in the scheduler queue. */

void scheduler_init();


uint32_t usr_get_fifo_len(app_fifo_t * p_fifo);

#endif