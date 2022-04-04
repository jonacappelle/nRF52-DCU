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
 *         File: usr_ble.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Bluetooth Low Energy (BLE) communication (MASTER)
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef __USR_BLE_H__
#define __USR_BLE_H__

#include "ble_imu_service_c.h"
#include "usr_dfu.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app_fifo.h"
#include "nrf_drv_uart.h"
#include "app_scheduler.h"
#include "ble_advertising.h"
#include "usr_internal_comm.h"

#define CONNECTION   1
#define DISCONNECTION 0
#define INVALID_VALUE  0xFFFF

// Match connection handles to unique IDs
typedef struct
{
  uint16_t conn_handle;
  ble_gap_addr_t addr;
} dcu_connected_devices_t;


typedef struct batt
{
    uint8_t level;
    float voltage;
} BATTERY;

typedef struct batt_array
{
    BATTERY batt[NRF_SDH_BLE_CENTRAL_LINK_COUNT];
} BATTERY_ARRAY;

// Create a FIFO structure
typedef struct buffer
{
    app_fifo_t received_data_fifo;
    uint8_t received_data_buffer[4096];
} BUFFER;


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
    bool start_calibration;
} IMU;

//////////////////
// Initialization
//////////////////

// BLE Stack
void ble_stack_init(void);
// Discovery
void db_discovery_init(void);
// Gatt
void gatt_init(void);
// Services
void services_init();
// Scanning
void scan_init(void);
void scan_start(void);
void scan_stop(void);


// Buffers
void received_data_buffers_init();

// Scheduling
void schedule(app_sched_event_handler_t handler);
void imu_uart_sceduled(void *p_event_data, uint16_t event_size);

// Disconnect all BLE connections
void usr_ble_disconnect();

void usr_ble_handles_assign(ble_imu_service_c_t *p_ble_imu_service_c, ble_imu_service_c_evt_t *p_evt);
void usr_enable_notif(ble_imu_service_c_t *p_ble_imu_service_c, ble_imu_service_c_evt_t *p_evt);

// Synchronization enable/disable
void sync_enable();
void sync_disable();

// Set and get a whitelist of devices that may connect
void set_conn_dev_mask(dcu_conn_dev_t data[], uint8_t len);
void get_connected_devices(dcu_connected_devices_t* conn_dev, uint32_t len);

//////////////////
// Debugging
//////////////////

// Debugging functionality
void usr_ble_print_settings();
void usr_ble_print_connection_handles();
// Get length of connection handles
uint32_t usr_ble_get_conn_handle_len();

/////////////////////////////
// Data transfer / settings
/////////////////////////////

// Set configurations
void set_config_raw_enable(bool enable);
void set_config_sync_enable(bool enable);
void set_config_adc_enable(bool enable);
void set_config_gyro_enable(bool enable);
void set_config_accel_enable(bool enable);
void set_config_mag_enable(bool enable);
void set_config_quat6_enable(bool enable);
void set_config_quat9_enable(bool enable);
void set_config_euler_enable(bool enable);
void set_config_wom_enable(bool enable);
void set_config_frequency(uint32_t freq);
void set_config_start_calibration(bool enable);
void set_config_reset();

// Send buffered configuration to all sensors
uint32_t config_send();
void usr_ble_config_send(ble_imu_service_config_t config);
void ble_send_config(ble_imu_service_config_t * stop_config);

// Stop measurements
void config_send_stop();

// Disconnect IMU from specific connection handle
ret_code_t imu_disconnect(uint32_t conn_handle_num);

// Battery
void usr_batt_print_conn_handle();
void get_battery(BATTERY_ARRAY* batt, uint32_t* len);

#endif
