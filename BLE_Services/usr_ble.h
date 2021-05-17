#ifndef __USR_BLE_H__
#define __USR_BLE_H__


#include "ble_tes_c.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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



#endif