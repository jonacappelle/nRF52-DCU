
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



void libuarte_init();
void uart_event_handler(void * context, nrf_libuarte_async_evt_t * p_evt);

void uart_print(char msg[]);
uint8_t uart_rx_to_cmd(uint8_t *command_in, uint8_t len);

bool uart_in_progress();

ret_code_t uart_rx_buff_read(uint8_t * p_byte_array, uint32_t * p_size);
ret_code_t uart_rx_buff_get(uint8_t * p_byte);
void uart_queued_tx(uint8_t * data, uint32_t * len);

#endif