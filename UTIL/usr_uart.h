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
 *         File: usr_uart.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Non-blocking DMA-based UART driver with double buffering, based on libuarte
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef _USR_UART_H_
#define _USR_UART_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_libuarte_async.h"
#include "nrf_drv_clock.h"
#include <bsp.h>
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_queue.h"

// Application scheduler
#include "app_scheduler.h"

#include "settings.h"

// UART commands
#define CMD_GYRO 0x67  //g
#define CMD_ACCEL 0x61 //a
#define CMD_MAG 0x6D   //m

#define CMD_QUAT 0x71  //q
#define CMD_QUAT6 0x36 //6
#define CMD_QUAT9 0x39 //9

#define CMD_CR 0x0A //carriage return

#define CMD_FREQ 0x66 //f

#define CMD_FREQ_LEN 3 // Length of the frequency component of uart config
#define CMD_FREQ_10 10
#define CMD_FREQ_50 50
#define CMD_FREQ_100 100
#define CMD_FREQ_225 225
#define CMD_FREQ_200 200

#define CMD_PRINT 0x68    //h
#define CMD_SETTINGS 0x70 //p
#define CMD_RESET 0x72    //r
#define CMD_SEND 0x73     //s
#define CMD_DISCONNECT 0x64 //d

#define CMD_WOM 0x77 //w

#define CMD_SYNC 0x69         //i
#define CMD_SYNC_ENABLE 0x31  //1
#define CMD_SYNC_DISABLE 0x30 //0

#define CMD_CALIBRATION 0x63 //c

#define CMD_ADC 0x65 //e

#define CMD_LIST 0x6c //l

#define CMD_BATT    0x62 //b

// Initialization
void libuarte_init(app_sched_event_handler_t scheduled_function);
// UART event handler
void uart_event_handler(void * context, nrf_libuarte_async_evt_t * p_evt);

// Uart transmitting
void uart_print(char msg[]);
void uart_queued_tx(uint8_t * data, uint32_t * len);

// Conversions
uint32_t uart_rx_to_cmd(uint8_t *command_in, uint8_t len);

// State check
bool uart_in_progress();

// Buffering
ret_code_t uart_rx_buff_read(uint8_t * p_byte_array, uint32_t * p_size);
ret_code_t uart_rx_buff_get(uint8_t * p_byte);

#endif
