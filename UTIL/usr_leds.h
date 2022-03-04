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
 *         File: usr_leds.c
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: On-board LED drivers
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef _USR_UART_H_
#define _USR_UART_H_

#include "usr_ble.h"

// Initialization
void usr_gpio_init();

// Animation on start-up
void leds_startup(void);

// Timer functionality for driving LEDs
void create_timers();

// Set correct LEDs on DCU
void DCU_set_connection_leds(dcu_connected_devices_t evt[], uint8_t state);

// CPU activity pin toggling
void check_cpu_activity();

#endif