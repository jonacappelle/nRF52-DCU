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


void comm_process(ble_imu_service_c_evt_type_t type, ble_imu_service_c_evt_t * data_in);
void comm_rx_process(void *p_event_data, uint16_t event_size);
static uint8_t calculate_cs(uint8_t * data, uint32_t * len);
static void check_buffer_overflow(uint32_t* data_len);
static void check_not_negative_uint8(uint8_t* data);
void comm_send_ok(command_type_byte_t command_type);

void set_stm32_real_time(stm32_time_t time, uint32_t this_offset);
stm32_time_t get_stm32_real_time();
stm32_time_t calculate_total_time(stm32_time_t local_time);

// void uart_send_conn_dev(dcu_connected_devices_t* dev, uint32_t len);
// void uart_send_conn_dev_update(ble_gap_addr_t* dev, uint32_t len, command_type_conn_dev_update_byte_t state);
#endif