#ifndef __MAIN_H__
#define __MAIN_H__

#include "settings.h"

// Includes
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Own packages
#include "nrf_delay.h"
#include "ble_conn_state.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"

// TimeSync
#include "time_sync.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "usr_time_sync.h"

// UARTE using easyDMA
#include "nrf_drv_uart.h"
#include "app_fifo.h"

// Application scheduler
#include "app_scheduler.h"

// IMU Params
#include "imu_params.h"

// Receive data from Thingy motion service
#include "ble_imu_service_c.h"

// List of connected slaves
#include "sdk_mapped_flags.h"

// Utilities
#include "usr_util.h"

// Libuarte - a more advanced UART driver
#include "usr_uart.h"

// Bluetooth functionality
#include "usr_ble.h"


// Struct to keep track of received data





#endif