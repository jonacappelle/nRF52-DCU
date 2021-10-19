#ifndef _USR_UTIL_H__
#define _USR_UTIL_H__


#define PIN_CPU_ACTIVITY    19

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app_timer.h"
#include "app_fifo.h"

#include "usr_ble.h"

void uart_rx_scheduled(void *p_event_data, uint16_t event_size);

void idle_state_handle(void);
void power_management_init(void);
void log_init(void);

void usr_gpio_init();

void check_cpu_activity();

uint32_t calculate_string_len(char * string);

void buttons_leds_init(void);


// APP Scheduler
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 200                                      /**< Maximum number of events in the scheduler queue. */

void scheduler_init();


uint32_t usr_get_fifo_len(app_fifo_t * p_fifo);

float usr_map_adc_to_uint8(uint8_t lvl);
uint8_t usr_adc_voltage_to_percent(float voltage);

void leds_startup(void);
void create_timers();
void DCU_set_connection_leds(dcu_connected_devices_t evt[], uint8_t state);

void check_reset_reason();
void clocks_start(void);

void timer_init(void);

#endif