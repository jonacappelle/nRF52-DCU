#ifndef __USR_BLE_H__
#define __USR_BLE_H__


#include "ble_tes_c.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app_fifo.h"

#include "nrf_drv_uart.h"

#include "app_scheduler.h"

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
} IMU;

void timer_init(void);

void received_data_buffers_init();

void schedule(app_sched_event_handler_t handler);
void imu_uart_sceduled(void *p_event_data, uint16_t event_size);

void ble_send_config(ble_tes_config_t * stop_config);

void usr_ble_config_send(ble_tes_config_t config);
void ble_send_config(ble_tes_config_t * stop_config);

void usr_ble_disconnect();

void usr_ble_handles_assign(ble_thingy_tes_c_t *p_ble_tes_c, ble_tes_c_evt_t *p_evt);

void usr_enable_notif(ble_thingy_tes_c_t *p_ble_tes_c, ble_tes_c_evt_t *p_evt);

void db_discovery_init(void);
void ble_stack_init(void);
void gatt_init(void);
void services_init();
void scan_init(void);
void scan_start(void);

void usr_ble_print_settings();
void usr_ble_print_connection_handles();

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
void set_config_reset();
void config_send();

#endif