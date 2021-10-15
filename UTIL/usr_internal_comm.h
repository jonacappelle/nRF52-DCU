#ifndef _USR_INTERNAL_COMM_H__
#define _USR_INTERNAL_COMM_H__


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ble_imu_service_c.h"


void comm_process(ble_imu_service_c_evt_type_t type, ble_imu_service_c_evt_t * data_in);
void comm_rx_process(void *p_event_data, uint16_t event_size);
static uint8_t calculate_cs(uint8_t * data, uint32_t * len);

#endif