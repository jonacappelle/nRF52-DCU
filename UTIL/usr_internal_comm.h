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
 *         File: usr_internal_comm.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Communication between nRF52 and STM32
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef _USR_INTERNAL_COMM_H__
#define _USR_INTERNAL_COMM_H__


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ble_imu_service_c.h"
#include "internal_comm_protocol.h"

// #include "usr_ble.h"

typedef struct
{
    uint8_t addr[BLE_GAP_ADDR_LEN]; /**< 48-bit address, LSB format. */
} dcu_conn_dev_t;

typedef uint64_t stm32_time_t;

// Process data received by BLE service
void comm_process(ble_imu_service_c_evt_type_t type, ble_imu_service_c_evt_t * data_in);

// Send ACK to STM32
void comm_send_ok(command_type_byte_t command_type);

// Event hander for RX data
void comm_rx_process(void *p_event_data, uint16_t event_size);

// CS calculation
static uint8_t calculate_cs(uint8_t * data, uint32_t * len);

// Error handling
static void check_buffer_overflow(uint32_t* data_len);
static void check_not_negative_uint8(uint8_t* data);

// Time synchronization between nRF52 and STM32
void set_stm32_real_time(stm32_time_t time, uint32_t this_offset);
stm32_time_t get_stm32_real_time();
stm32_time_t calculate_total_time(stm32_time_t local_time);

// void uart_send_conn_dev(dcu_connected_devices_t* dev, uint32_t len);
void uart_send_conn_dev_update(ble_gap_addr_t* dev, uint32_t len, command_type_conn_dev_update_byte_t state);
#endif