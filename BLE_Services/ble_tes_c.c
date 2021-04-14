/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include "sdk_common.h"
// #if NRF_MODULE_ENABLED(BLE_TES_C)
// #ifdef BLE_TES_C_ENABLED

#include "ble_tes_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"
#define NRF_LOG_MODULE_NAME ble_tes_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define TX_BUFFER_MASK         0x07                  /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */
#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */

typedef enum
{
    READ_REQ,  /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ  /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of this message, i.e. read or write message. */
    union
    {
        uint16_t       read_handle;  /**< Read request message. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;


static tx_message_t m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t     m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t     m_tx_index = 0;               /**< Current index in the transmit buffer from where the next message to be transmitted resides. */


/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("SD Read/Write API returns Success..");
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
            NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be "
                "attempted again..");
        }
    }
}


/**@brief Function for handling write response events.
 *
 * @param[in] p_ble_tes_c Pointer to the Thingy Enviroment Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_write_rsp(ble_thingy_tes_c_t * p_ble_tes_c, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ble_tes_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if there is any message to be sent across to the peer and send it.
    tx_buffer_process();
}


/**@brief Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details This function will uses the Handle Value Notification received from the SoftDevice
 *          and checks if it is a notification of Button state from the peer. If
 *          it is, this function will decode the state of the button and send it to the
 *          application.
 *
 * @param[in] p_ble_tes_c Pointer to the Thingy Enviroment Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_thingy_tes_c_t * p_ble_tes_c, ble_evt_t const * p_ble_evt)
{

    // Check if the event is on the link for this instance
    if (p_ble_tes_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    
    ble_tes_c_evt_t ble_tes_c_evt;

    ble_tes_c_evt.conn_handle                = p_ble_tes_c->conn_handle;
    
    
    // Check if this is a Temperature notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_tes_c->peer_thingy_tes_db.quat_handle)
    {
        ble_tes_c_evt.evt_type = BLE_TMS_EVT_QUAT;
        ble_tes_c_evt.params.value.quat_data = *(ble_tms_quat_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;

        NRF_LOG_DEBUG("quat: %d", ble_tes_c_evt.params.value.quat_data.w);
    } 
    // else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_tes_c->peer_tes_db.pressure_handle)
    // {
    //     ble_tes_c_evt.evt_type = BLE_TES_C_EVT_PRESSURE_NOTIFICATION;
    //     ble_tes_c_evt.params.value.pressure_data = *(ble_tes_pressure_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    // }
    // else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_tes_c->peer_tes_db.humidity_handle)
    // {
    //     ble_tes_c_evt.evt_type = BLE_TES_C_EVT_HUMIDITY_NOTIFICATION;
    //     ble_tes_c_evt.params.value.humidity_data = *(ble_tes_humidity_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    // }
    // else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_tes_c->peer_tes_db.gas_handle)
    // {
    //     ble_tes_c_evt.evt_type = BLE_TES_C_EVT_GAS_NOTIFICATION;
    //     ble_tes_c_evt.params.value.gas_data = *(ble_tes_gas_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    // }
    // else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_tes_c->peer_tes_db.color_handle)
    // {
    //     ble_tes_c_evt.evt_type = BLE_TES_C_EVT_COLOR_NOTIFICATION;
    //     ble_tes_c_evt.params.value.color_data = *(ble_tes_color_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    // }
    // else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_tes_c->peer_tes_db.config_handle)
    // {
    //     ble_tes_c_evt.evt_type = BLE_TES_C_EVT_CONFIG_NOTIFICATION;
    // } else return;

    p_ble_tes_c->evt_handler(p_ble_tes_c, &ble_tes_c_evt);
}


/**@brief Function for handling Disconnected event received from the SoftDevice.
 *
 * @details This function check if the disconnect event is happening on the link
 *          associated with the current instance of the module, if so it will set its
 *          conn_handle to invalid.
 *
 * @param[in] p_ble_tes_c Pointer to the Thingy Enviroment Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_thingy_tes_c_t * p_ble_tes_c, ble_evt_t const * p_ble_evt)
{
    if (p_ble_tes_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_tes_c->conn_handle                            = BLE_CONN_HANDLE_INVALID;

        p_ble_tes_c->peer_thingy_tes_db.config_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.config_handle       = BLE_GATT_HANDLE_INVALID;

        p_ble_tes_c->peer_thingy_tes_db.euler_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.heading_cccd_handle       = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.orient_cccd_handle       = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.pedometer_cccd_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.quat_cccd_handle          = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.raw_cccd_handle         = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.rot_cccd_handle         = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.tap_cccd_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.gravity_cccd_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.euler_handle                 = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.heading_handle               = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.orient_handle              = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.pedometer_handle         = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.quat_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.raw_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.rot_handle                 = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.tap_handle               = BLE_GATT_HANDLE_INVALID;
        p_ble_tes_c->peer_thingy_tes_db.gravity_handle              = BLE_GATT_HANDLE_INVALID;
    }
}


void ble_thingy_tes_on_db_disc_evt(ble_thingy_tes_c_t * p_ble_thingy_tes_c, ble_db_discovery_evt_t const * p_evt)
{

    NRF_LOG_DEBUG("ble_thingy_tes_on_db_disc_evt");

    // Check if the Led Button Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == THINGY_TES_UUID_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == p_ble_thingy_tes_c->uuid_type)
    {
        ble_tes_c_evt_t evt;
        evt.evt_type    = BLE_THINGY_TES_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;
        //NRF_LOG_INFO("tes discover count : %d", p_evt->params.discovered_db.char_count);
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            const ble_gatt_db_char_t * p_char = &(p_evt->params.discovered_db.charateristics[i]);
            NRF_LOG_INFO("<i=%d> UUID : %x", i, p_char->characteristic.uuid.uuid);
            switch (p_char->characteristic.uuid.uuid)
            {
                case THINGY_TES_UUID_CONFIG_CHAR:
                    evt.params.peer_db.config_cccd_handle      = p_char->cccd_handle;
                    evt.params.peer_db.config_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("401 evt CONFIG handle = %d", p_char->characteristic.handle_value);
                    break;                    
                case THINGY_TES_UUID_TAP_CHAR:
                    evt.params.peer_db.tap_cccd_handle      = p_char->cccd_handle;
                    evt.params.peer_db.tap_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("402 evt TAP handle = %d", p_char->characteristic.handle_value);
                    break;
                case THINGY_TES_UUID_ORIENTATION_CHAR: 
                    evt.params.peer_db.orient_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.orient_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("403 evt Orientation handle = %d", evt.params.peer_db.orient_handle);
                    break;
                case THINGY_TES_UUID_QUATERNION_CHAR:
                    evt.params.peer_db.quat_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.quat_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("404 evt Quaternion handle = %d", evt.params.peer_db.quat_handle);
                case THINGY_TES_UUID_PEDOMETER_CHAR: 
                    evt.params.peer_db.pedometer_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.pedometer_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("405 evt step counter handle = %d", evt.params.peer_db.pedometer_handle);
                case THINGY_TES_UUID_RAW_CHAR:
                    evt.params.peer_db.raw_cccd_handle      = p_char->cccd_handle;
                    evt.params.peer_db.raw_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("406 evt raw data handle = %d", evt.params.peer_db.raw_handle);
                    break;
                case THINGY_TES_UUID_EULER_CHAR: 
                    evt.params.peer_db.euler_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.euler_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("407 evt euler handle = %d", evt.params.peer_db.euler_handle);
                    break;
                case THINGY_TES_UUID_ROT_MAT_CHAR: 
                    evt.params.peer_db.rot_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.rot_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("408 evt rotation matrix handle = %d", evt.params.peer_db.rot_handle);
                case THINGY_TES_UUID_HEADING_CHAR:
                    evt.params.peer_db.heading_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.heading_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("409 evt heading handle = %d", evt.params.peer_db.heading_handle);
                case THINGY_TES_UUID_GRAVITY_CHAR: 
                    evt.params.peer_db.gravity_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.gravity_handle = p_char->characteristic.handle_value;
                    NRF_LOG_INFO("410 evt gravity handle = %d", evt.params.peer_db.gravity_handle);
                default:
                    break;
            }
        }
        //If the instance has been assigned prior to db_discovery, assign the db_handles
         if (p_ble_thingy_tes_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if (
                (p_ble_thingy_tes_c->peer_thingy_tes_db.config_cccd_handle    = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.config_handle         = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.euler_cccd_handle    = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.euler_handle         = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.pedometer_cccd_handle = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.pedometer_handle     = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.quat_cccd_handle     = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.quat_handle          = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.raw_cccd_handle      = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.raw_handle           = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.tap_cccd_handle      = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.tap_handle           = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.heading_cccd_handle  = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.heading_handle       = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.orient_cccd_handle   = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.orient_handle        = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.rot_cccd_handle      = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_thingy_tes_c->peer_thingy_tes_db.rot_handle           = BLE_GATT_HANDLE_INVALID))
            {
                p_ble_thingy_tes_c->peer_thingy_tes_db = evt.params.peer_db;
            }
        }
        p_ble_thingy_tes_c->evt_handler(p_ble_thingy_tes_c, &evt);

    }
    else if(p_evt->evt_type != BLE_DB_DISCOVERY_COMPLETE)
    {
        NRF_LOG_DEBUG("Ble service discovery not complete");

        ble_tes_c_evt_t evt;
        evt.evt_type    = 0;
        p_ble_thingy_tes_c->evt_handler(p_ble_thingy_tes_c, &evt);
    }
}


uint32_t ble_thingy_tes_c_init(ble_thingy_tes_c_t * p_ble_thingy_tes_c, ble_thingy_tes_c_init_t * p_ble_thingy_tes_c_init)
{
    uint32_t      err_code;
    ble_uuid_t    thingy_tes_uuid;
    ble_uuid128_t thingy_tes_base_uuid = {THINGY_UUID_BASE};

    VERIFY_PARAM_NOT_NULL(p_ble_thingy_tes_c);
    VERIFY_PARAM_NOT_NULL(p_ble_thingy_tes_c_init);
    VERIFY_PARAM_NOT_NULL(p_ble_thingy_tes_c_init->evt_handler);

    p_ble_thingy_tes_c->peer_thingy_tes_db.config_cccd_handle      = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.config_handle           = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.euler_cccd_handle      = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.euler_handle           = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.heading_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.heading_handle         = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.orient_cccd_handle     = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.orient_handle          = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.pedometer_cccd_handle  = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.pedometer_handle       = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.quat_cccd_handle       = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.quat_handle            = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.raw_cccd_handle        = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.raw_handle             = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.rot_cccd_handle        = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.rot_handle             = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.tap_cccd_handle        = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.tap_handle             = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.gravity_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    p_ble_thingy_tes_c->peer_thingy_tes_db.gravity_handle         = BLE_GATT_HANDLE_INVALID;

    p_ble_thingy_tes_c->conn_handle                    = BLE_CONN_HANDLE_INVALID;
    p_ble_thingy_tes_c->evt_handler                    = p_ble_thingy_tes_c_init->evt_handler;

    err_code = sd_ble_uuid_vs_add(&thingy_tes_base_uuid, &p_ble_thingy_tes_c->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);

    thingy_tes_uuid.type = p_ble_thingy_tes_c->uuid_type;
    thingy_tes_uuid.uuid = THINGY_TES_UUID_SERVICE;
    return ble_db_discovery_evt_register(&thingy_tes_uuid);
}

void ble_tes_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{

    NRF_LOG_DEBUG("ble_tes_c_on_ble_evt");

    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_thingy_tes_c_t * p_ble_tes_c = (ble_thingy_tes_c_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_tes_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_tes_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_tes_c, p_ble_evt);
            break;

        default:
            break;
    }
}


/**@brief Function for configuring the CCCD.
 *
 * @param[in] conn_handle The connection handle on which to configure the CCCD.
 * @param[in] handle_cccd The handle of the CCCD to be configured.
 * @param[in] enable      Whether to enable or disable the CCCD.
 *
 * @return NRF_SUCCESS if the CCCD configure was successfully sent to the peer.
 */
static uint32_t cccd_configure_tes(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{
    NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
        handle_cccd,conn_handle);

    tx_message_t * p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    
    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}


uint32_t ble_tes_c_quaternion_notif_enable(ble_thingy_tes_c_t * p_ble_tes_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_tes_c);

    if (p_ble_tes_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure_tes(p_ble_tes_c->conn_handle,
                          p_ble_tes_c->peer_thingy_tes_db.quat_cccd_handle,
                          true);
}

uint32_t ble_tes_c_handles_assign(ble_thingy_tes_c_t    * p_ble_tes_c,
                                  uint16_t         conn_handle,
                                  const tes_db_t * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_tes_c);

    p_ble_tes_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_tes_c->peer_thingy_tes_db = *p_peer_handles;
    }
    return NRF_SUCCESS;
}

// #endif // NRF_MODULE_ENABLED(BLE_TES_C)
