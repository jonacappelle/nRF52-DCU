#ifndef _USR_INTERNAL_COMM_H__
#define _USR_INTERNAL_COMM_H__


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ble_imu_service_c.h"


void comm_process(ble_imu_service_c_evt_type_t type, ble_imu_service_c_evt_t * data_in);


#endif