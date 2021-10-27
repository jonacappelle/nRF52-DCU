#ifndef _USR_INTERNAL_COMM_H__
#define _USR_INTERNAL_COMM_H__


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ble_imu_service_c.h"

typedef struct
{
    uint8_t addr[BLE_GAP_ADDR_LEN]; /**< 48-bit address, LSB format. */
} dcu_conn_dev_t;


void comm_process(ble_imu_service_c_evt_type_t type, ble_imu_service_c_evt_t * data_in);
void comm_rx_process(void *p_event_data, uint16_t event_size);
static uint8_t calculate_cs(uint8_t * data, uint32_t * len);
static void check_buffer_overflow(uint32_t* data_len);
static void check_not_negative_uint8(uint8_t* data);

#endif