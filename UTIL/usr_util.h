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
 *         File: usr_util.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Utilities
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef _USR_UTIL_H__
#define _USR_UTIL_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app_timer.h"
#include "app_fifo.h"
#include "usr_ble.h"

// APP Scheduler
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 200                                      /**< Maximum number of events in the scheduler queue. */
void scheduler_init();

// Power management
void idle_state_handle(void);
void power_management_init(void);

// Segger RTT logging
void log_init(void);

// GPIO
void usr_gpio_init();
void check_cpu_activity();

// LEDs
void buttons_leds_init(void);
void leds_startup(void);
void DCU_set_connection_leds(dcu_connected_devices_t evt[], uint8_t state);

// Timers
void timer_init(void);
void create_timers();

// Clocking
void clocks_start(void);

// UART utils
void uart_rx_scheduled(void *p_event_data, uint16_t event_size);
uint32_t calculate_string_len(char * string);

// Buffering
uint32_t usr_get_fifo_len(app_fifo_t * p_fifo);

// Battery
float usr_map_adc_to_uint8(uint8_t lvl);
uint8_t usr_adc_voltage_to_percent(float voltage);

// Debugging
void check_reset_reason();

#endif
