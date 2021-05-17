#ifndef __MAIN_H__
#define __MAIN_H__



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
#include "ble_tes_c.h"

// List of connected slaves
#include "sdk_mapped_flags.h"

// Utilities
#include "usr_util.h"

// Libuarte - a more advanced UART driver
#include "usr_uart.h"

// Bluetooth functionality
#include "usr_ble.h"


// Structs

// Create a FIFO structure
typedef struct buffer
{
    app_fifo_t received_data_fifo;
    uint8_t received_data_buffer[4096];
} BUFFER;


// Initialisation of struct to keep track of different buffers
BUFFER buffer;

// Struct to keep track of received data

typedef struct gyro
{
    float x;
    float y;
    float z;
} gyro_t;

typedef struct accel
{
    float x;
    float y;
    float z;
} accel_t;

typedef struct mag
{
    float x;
    float y;
    float z;
} mag_t;

typedef struct raw_data
{
    gyro_t gryo;
    accel_t accel;
    mag_t mag;
} raw_data_t;

typedef struct quat_data
{
    float w;
    float x;
    float y;
    float z;
} quat_data_t;

typedef struct adc_data
{
    uint32_t raw[40];
} adc_data_t;

typedef struct received_data
{
    bool raw_data_present;
    bool quat_data_present;
    bool adc_data_present;
    raw_data_t raw_data;
    quat_data_t quat_data;
    // adc_data_t adc_data;
    uint16_t conn_handle;
} received_data_t;

typedef struct imu
{
    bool gyro_enabled;
    bool accel_enabled;
    bool mag_enabled;
    bool quat6_enabled;
    bool quat9_enabled;
    bool euler_enabled;
    bool wom;
    bool stop;
    bool sync_enabled;
    uint64_t sync_start_time;
    uint32_t frequency; // period in milliseconds (ms)
    uint16_t packet_length;
    int received_packet_counter1;
    int received_packet_counter2;
    int received_packet_counter3;
    int received_packet_counter4;
    nrf_drv_uart_t uart;
    uint32_t evt_scheduled;
    uint32_t uart_rx_evt_scheduled;
    bool adc;
} IMU;

// Initialisation of IMU struct
IMU imu = {
    .frequency = 0,
    .stop = 0,
    .sync_start_time = 0,
    .received_packet_counter1 = 0,
    .received_packet_counter2 = 0,
    .received_packet_counter3 = 0,
    .received_packet_counter4 = 0,
    .evt_scheduled = 0,
    .uart_rx_evt_scheduled = 0,
    .uart = NRF_DRV_UART_INSTANCE(0),
};



#endif