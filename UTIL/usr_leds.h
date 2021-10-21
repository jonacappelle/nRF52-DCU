#ifndef _USR_UART_H_
#define _USR_UART_H_

#include "usr_ble.h"


void usr_gpio_init();

void leds_startup(void);
void create_timers();
void DCU_set_connection_leds(dcu_connected_devices_t evt[], uint8_t state);
void check_cpu_activity();


#endif