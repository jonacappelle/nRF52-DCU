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
// #if NRF_MODULE_ENABLED(BLE_IMU_SERVICE_C)
// #ifdef BLE_IMU_SERVICE_C_ENABLED

#include "ble_imu_service_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"
#define NRF_LOG_MODULE_NAME ble_imu_service_c
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
// static void tx_buffer_process(void)
// {
//     if (m_tx_index != m_tx_insert_index)
//     {
//         uint32_t err_code;

//         if (m_tx_buffer[m_tx_index].type == READ_REQ)
//         {
//             err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
//                                          m_tx_buffer[m_tx_index].req.read_handle,
//                                          0);
//         }
//         else
//         {
//             err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
//                                           &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
//         }
//         if (err_code == NRF_SUCCESS)
//         {
//             NRF_LOG_DEBUG("SD Read/Write API returns Success..");
//             m_tx_index++;
//             m_tx_index &= TX_BUFFER_MASK;
//         }
//         else
//         {
//             NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be attempted again..");
//         }
//     }
// }


/**@brief Function for intercepting errors of GATTC and BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
    // ble_imu_service_c_t * p_tes_c = (ble_imu_service_c_t *)p_ctx;

    NRF_LOG_INFO("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

    // if (p_tes_c->error_handler != NULL)
    // {
    //     p_tes_c->error_handler(nrf_error);
    // }
}



/**@brief Function for handling write response events.
 *
 * @param[in] p_ble_imu_service_c Pointer to the Thingy Enviroment Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_write_rsp(ble_imu_service_c_t * p_ble_imu_service_c, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ble_imu_service_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
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
 * @param[in] p_ble_imu_service_c Pointer to the Thingy Enviroment Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_imu_service_c_t * p_ble_imu_service_c, ble_evt_t const * p_ble_evt)
{

    // Check if the event is on the link for this instance
    if (p_ble_imu_service_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    
    ble_imu_service_c_evt_t ble_imu_service_c_evt;

    ble_imu_service_c_evt.conn_handle                = p_ble_imu_service_c->conn_handle;
    
    
    // Check if this is a Quaternion notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_imu_service_c->peer_imu_service_db.quat_handle)
    {
        ble_imu_service_c_evt.evt_type = BLE_IMU_SERVICE_EVT_QUAT;
        ble_imu_service_c_evt.params.value.quat_data = *(ble_imu_service_quat_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    } 
    // Check if this is a info notification.
    else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_imu_service_c->peer_imu_service_db.info_handle)
    {
        ble_imu_service_c_evt.evt_type = BLE_IMU_SERVICE_EVT_INFO;
        ble_imu_service_c_evt.params.value.info_data = *(ble_imu_service_info_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    }
    // Check if this is a Euler angle notification.
    else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_imu_service_c->peer_imu_service_db.euler_handle)
    {
        ble_imu_service_c_evt.evt_type = BLE_IMU_SERVICE_EVT_EULER;
        ble_imu_service_c_evt.params.value.euler_data = *(ble_imu_service_euler_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    }
    // Check if this is a Raw data notification.
    else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_imu_service_c->peer_imu_service_db.raw_handle)
    {
        ble_imu_service_c_evt.evt_type = BLE_IMU_SERVICE_EVT_RAW;
        ble_imu_service_c_evt.params.value.raw_data = *(ble_imu_service_raw_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    }
    else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_imu_service_c->peer_imu_service_db.adc_handle)
    {
        ble_imu_service_c_evt.evt_type = BLE_IMU_SERVICE_EVT_ADC;
        ble_imu_service_c_evt.params.value.adc_data = *(ble_imu_service_adc_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    }
    // else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_imu_service_c->peer_imu_service_db.gas_handle)
    // {
    //     ble_imu_service_c_evt.evt_type = BLE_IMU_SERVICE_C_EVT_GAS_NOTIFICATION;
    //     ble_imu_service_c_evt.params.value.gas_data = *(ble_imu_service_gas_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    // }
    // else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_imu_service_c->peer_imu_service_db.color_handle)
    // {
    //     ble_imu_service_c_evt.evt_type = BLE_IMU_SERVICE_C_EVT_COLOR_NOTIFICATION;
    //     ble_imu_service_c_evt.params.value.color_data = *(ble_imu_service_color_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
    // }
    // else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_imu_service_c->peer_imu_service_db.config_handle)
    // {
    //     ble_imu_service_c_evt.evt_type = BLE_IMU_SERVICE_C_EVT_CONFIG_NOTIFICATION;
    // } else return;

    else return;

    p_ble_imu_service_c->evt_handler(p_ble_imu_service_c, &ble_imu_service_c_evt);
}


/**@brief Function for handling Disconnected event received from the SoftDevice.
 *
 * @details This function check if the disconnect event is happening on the link
 *          associated with the current instance of the module, if so it will set its
 *          conn_handle to invalid.
 *
 * @param[in] p_ble_imu_service_c Pointer to the Thingy Enviroment Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_imu_service_c_t * p_ble_imu_service_c, ble_evt_t const * p_ble_evt)
{
    if (p_ble_imu_service_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_imu_service_c->conn_handle                            = BLE_CONN_HANDLE_INVALID;

        p_ble_imu_service_c->peer_imu_service_db.config_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.config_handle       = BLE_GATT_HANDLE_INVALID;

        p_ble_imu_service_c->peer_imu_service_db.euler_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.heading_cccd_handle       = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.adc_cccd_handle       = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.pedometer_cccd_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.quat_cccd_handle          = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.info_cccd_handle          = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.raw_cccd_handle         = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.rot_cccd_handle         = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.tap_cccd_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.gravity_cccd_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.euler_handle                 = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.heading_handle               = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.adc_handle              = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.pedometer_handle         = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.quat_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.info_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.raw_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.rot_handle                 = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.tap_handle               = BLE_GATT_HANDLE_INVALID;
        p_ble_imu_service_c->peer_imu_service_db.gravity_handle              = BLE_GATT_HANDLE_INVALID;
    }
}


void ble_imu_service_on_db_disc_evt(ble_imu_service_c_t * p_ble_imu_service_c, ble_db_discovery_evt_t const * p_evt)
{

    NRF_LOG_DEBUG("ble_imu_service_on_db_disc_evt");

    // Check if the Led Button Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == IMU_SERVICE_UUID_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == p_ble_imu_service_c->uuid_type)
    {
        ble_imu_service_c_evt_t evt;
        evt.evt_type    = BLE_IMU_SERVICE_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;
        NRF_LOG_DEBUG("tes discover count : %d", p_evt->params.discovered_db.char_count);
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            const ble_gatt_db_char_t * p_char = &(p_evt->params.discovered_db.charateristics[i]);
            // NRF_LOG_INFO("<i=%d> UUID : %x", i, p_char->characteristic.uuid.uuid);
            switch (p_char->characteristic.uuid.uuid)
            {
                case IMU_SERVICE_UUID_CONFIG_CHAR:
                    evt.params.peer_db.config_cccd_handle      = p_char->cccd_handle;
                    evt.params.peer_db.config_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("401 evt CONFIG handle = %d", p_char->characteristic.handle_value);
                    break;                    
                case IMU_SERVICE_UUID_TAP_CHAR:
                    evt.params.peer_db.tap_cccd_handle      = p_char->cccd_handle;
                    evt.params.peer_db.tap_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("402 evt TAP handle = %d", p_char->characteristic.handle_value);
                    break;
                case IMU_SERVICE_UUID_ADC_CHAR: 
                    evt.params.peer_db.adc_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.adc_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("403 evt ADC handle = %d", evt.params.peer_db.adc_handle);
                    break;
                case IMU_SERVICE_UUID_QUATERNION_CHAR:
                    evt.params.peer_db.quat_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.quat_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("404 evt Quaternion handle = %d", evt.params.peer_db.quat_handle);
                case IMU_SERVICE_UUID_INFO_CHAR:
                    evt.params.peer_db.info_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.info_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("404 evt Info handle = %d", evt.params.peer_db.info_handle);
                case IMU_SERVICE_UUID_PEDOMETER_CHAR: 
                    evt.params.peer_db.pedometer_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.pedometer_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("405 evt step counter handle = %d", evt.params.peer_db.pedometer_handle);
                case IMU_SERVICE_UUID_RAW_CHAR:
                    evt.params.peer_db.raw_cccd_handle      = p_char->cccd_handle;
                    evt.params.peer_db.raw_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("406 evt raw data handle = %d", evt.params.peer_db.raw_handle);
                    break;
                case IMU_SERVICE_UUID_EULER_CHAR: 
                    evt.params.peer_db.euler_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.euler_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("407 evt euler handle = %d", evt.params.peer_db.euler_handle);
                    break;
                case IMU_SERVICE_UUID_ROT_MAT_CHAR: 
                    evt.params.peer_db.rot_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.rot_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("408 evt rotation matrix handle = %d", evt.params.peer_db.rot_handle);
                case IMU_SERVICE_UUID_HEADING_CHAR:
                    evt.params.peer_db.heading_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.heading_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("409 evt heading handle = %d", evt.params.peer_db.heading_handle);
                case IMU_SERVICE_UUID_GRAVITY_CHAR: 
                    evt.params.peer_db.gravity_cccd_handle = p_char->cccd_handle;
                    evt.params.peer_db.gravity_handle = p_char->characteristic.handle_value;
                    // NRF_LOG_INFO("410 evt gravity handle = %d", evt.params.peer_db.gravity_handle);
                default:
                    break;
            }
        }
        //If the instance has been assigned prior to db_discovery, assign the db_handles
         if (p_ble_imu_service_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            NRF_LOG_INFO("IMU_SERVICE instance has been initialized prior");
            if (
                (p_ble_imu_service_c->peer_imu_service_db.config_cccd_handle    = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.config_handle         = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.euler_cccd_handle    = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.euler_handle         = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.pedometer_cccd_handle = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.pedometer_handle     = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.quat_cccd_handle     = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.quat_handle          = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.info_cccd_handle     = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.info_handle          = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.raw_cccd_handle      = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.raw_handle           = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.tap_cccd_handle      = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.tap_handle           = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.heading_cccd_handle  = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.heading_handle       = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.adc_cccd_handle   = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.adc_handle        = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.rot_cccd_handle      = BLE_GATT_HANDLE_INVALID)&&
                (p_ble_imu_service_c->peer_imu_service_db.rot_handle           = BLE_GATT_HANDLE_INVALID))
            {
                p_ble_imu_service_c->peer_imu_service_db = evt.params.peer_db;
            }
        }
        p_ble_imu_service_c->evt_handler(p_ble_imu_service_c, &evt);

    }
    else if(p_evt->evt_type != BLE_DB_DISCOVERY_COMPLETE)
    {
        NRF_LOG_DEBUG("Ble service discovery not complete");

        ble_imu_service_c_evt_t evt;
        evt.evt_type    = 0;
        p_ble_imu_service_c->evt_handler(p_ble_imu_service_c, &evt);
    }
}


uint32_t ble_imu_service_c_init(ble_imu_service_c_t * p_ble_imu_service_c, ble_imu_service_c_init_t * p_ble_imu_service_c_init)
{
    uint32_t      err_code;
    ble_uuid_t    imu_service_uuid;
    ble_uuid128_t imu_service_base_uuid = {THINGY_UUID_BASE};

    VERIFY_PARAM_NOT_NULL(p_ble_imu_service_c);
    VERIFY_PARAM_NOT_NULL(p_ble_imu_service_c_init);
    VERIFY_PARAM_NOT_NULL(p_ble_imu_service_c_init->evt_handler);

    p_ble_imu_service_c->peer_imu_service_db.config_cccd_handle      = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.config_handle           = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.euler_cccd_handle      = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.euler_handle           = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.heading_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.heading_handle         = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.adc_cccd_handle     = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.adc_handle          = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.pedometer_cccd_handle  = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.pedometer_handle       = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.quat_cccd_handle       = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.quat_handle            = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.info_cccd_handle       = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.info_handle            = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.raw_cccd_handle        = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.raw_handle             = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.rot_cccd_handle        = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.rot_handle             = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.tap_cccd_handle        = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.tap_handle             = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.gravity_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    p_ble_imu_service_c->peer_imu_service_db.gravity_handle         = BLE_GATT_HANDLE_INVALID;

    p_ble_imu_service_c->conn_handle                    = BLE_CONN_HANDLE_INVALID;
    p_ble_imu_service_c->evt_handler                    = p_ble_imu_service_c_init->evt_handler;
    p_ble_imu_service_c->p_gatt_queue               = p_ble_imu_service_c_init->p_gatt_queue;

    err_code = sd_ble_uuid_vs_add(&imu_service_base_uuid, &p_ble_imu_service_c->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);

    imu_service_uuid.type = p_ble_imu_service_c->uuid_type;
    imu_service_uuid.uuid = IMU_SERVICE_UUID_SERVICE;
    return ble_db_discovery_evt_register(&imu_service_uuid);
}

void ble_imu_service_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{

    // NRF_LOG_DEBUG("ble_imu_service_c_on_ble_evt");

    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_imu_service_c_t * p_ble_imu_service_c = (ble_imu_service_c_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_imu_service_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            // on_write_rsp(p_ble_imu_service_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_imu_service_c, p_ble_evt);
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
static uint32_t cccd_configure_tes(ble_imu_service_c_t * p_ble_imu_service_c, uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{
    // NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
    //     handle_cccd,conn_handle);

    // tx_message_t * p_msg;
    // uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    
    // p_msg              = &m_tx_buffer[m_tx_insert_index++];
    // m_tx_insert_index &= TX_BUFFER_MASK;

    // p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    // p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    // p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    // p_msg->req.write_req.gattc_params.offset   = 0;
    // p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    // p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    // p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    // p_msg->conn_handle                         = conn_handle;
    // p_msg->type                                = WRITE_REQ;

    // tx_buffer_process();
    // return NRF_SUCCESS;

///////////

    NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d", handle_cccd, conn_handle);

    nrf_ble_gq_req_t tes_c_req;
    uint8_t          cccd[BLE_CCCD_VALUE_LEN];
    uint16_t         cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    memset(&tes_c_req, 0, sizeof(tes_c_req));
 
    tes_c_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    tes_c_req.error_handler.cb            = gatt_error_handler;
    tes_c_req.error_handler.p_ctx         = p_ble_imu_service_c;
    tes_c_req.params.gattc_write.handle   = handle_cccd;
    tes_c_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
    tes_c_req.params.gattc_write.p_value  = cccd;
    tes_c_req.params.gattc_write.offset   = 0;
    tes_c_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;

    return nrf_ble_gq_item_add(p_ble_imu_service_c->p_gatt_queue, &tes_c_req, conn_handle);
}


uint32_t ble_imu_service_c_quaternion_notif_enable(ble_imu_service_c_t * p_ble_imu_service_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_imu_service_c);

    if (p_ble_imu_service_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure_tes(p_ble_imu_service_c, p_ble_imu_service_c->conn_handle,
                          p_ble_imu_service_c->peer_imu_service_db.quat_cccd_handle,
                          true);
}

uint32_t ble_imu_service_c_info_notif_enable(ble_imu_service_c_t * p_ble_imu_service_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_imu_service_c);

    if (p_ble_imu_service_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure_tes(p_ble_imu_service_c, p_ble_imu_service_c->conn_handle,
                          p_ble_imu_service_c->peer_imu_service_db.info_cccd_handle,
                          true);
}

uint32_t ble_imu_service_c_adc_notif_enable(ble_imu_service_c_t * p_ble_imu_service_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_imu_service_c);

    if (p_ble_imu_service_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure_tes(p_ble_imu_service_c, p_ble_imu_service_c->conn_handle,
                          p_ble_imu_service_c->peer_imu_service_db.adc_cccd_handle,
                          true);
}

uint32_t ble_imu_service_c_euler_notif_enable(ble_imu_service_c_t * p_ble_imu_service_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_imu_service_c);

    if (p_ble_imu_service_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure_tes(p_ble_imu_service_c, p_ble_imu_service_c->conn_handle,
                          p_ble_imu_service_c->peer_imu_service_db.euler_cccd_handle,
                          true);
}

uint32_t ble_imu_service_c_raw_notif_enable(ble_imu_service_c_t * p_ble_imu_service_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_imu_service_c);

    if (p_ble_imu_service_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure_tes(p_ble_imu_service_c, p_ble_imu_service_c->conn_handle,
                          p_ble_imu_service_c->peer_imu_service_db.raw_cccd_handle,
                          true);
}

uint32_t ble_imu_service_config_set(ble_imu_service_c_t * p_imu_service, ble_imu_service_config_t * p_data)
{
    uint16_t length = sizeof(ble_imu_service_config_t);

    VERIFY_PARAM_NOT_NULL(p_imu_service);

    if ((p_imu_service->conn_handle == BLE_CONN_HANDLE_INVALID))// || (!p_imu_service->is_tap_notif_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    NRF_LOG_INFO("Send config: conn_handle: %d", p_imu_service->conn_handle)

    if (length > BLE_IMU_SERVICE_MAX_DATA_LEN)
    {
        NRF_LOG_WARNING("Content too long.");
        return NRF_ERROR_INVALID_PARAM;
    }

    // ble_gattc_write_params_t write_params;

    // memset(&write_params, 0, sizeof(write_params));

    // write_params.write_op = BLE_GATT_OP_WRITE_REQ;
    // write_params.handle = p_imu_service->peer_imu_service_db.config_handle;
    // write_params.len = length;
    // write_params.offset = 0;
    // write_params.p_value = (uint8_t *)p_data;
    // write_params.flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    // return sd_ble_gattc_write(p_imu_service->conn_handle, &write_params);

    nrf_ble_gq_req_t write_req;

    memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

    write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    write_req.error_handler.cb            = gatt_error_handler;
    write_req.error_handler.p_ctx         = p_imu_service;
    write_req.params.gattc_write.handle   = p_imu_service->peer_imu_service_db.config_handle;
    write_req.params.gattc_write.len      = length;
    write_req.params.gattc_write.offset   = 0;
    write_req.params.gattc_write.p_value  = (uint8_t *)p_data;
    write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
    // write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_CMD;
    write_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    ret_code_t err_code;
    err_code =  nrf_ble_gq_item_add(p_imu_service->p_gatt_queue, &write_req, p_imu_service->conn_handle);
    return err_code;


    // tx_message_t * p_msg;
    // // uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    
    // p_msg              = &m_tx_buffer[m_tx_insert_index++];
    // m_tx_insert_index &= TX_BUFFER_MASK;

    // p_msg->req.write_req.gattc_params.handle   = p_imu_service->peer_imu_service_db.config_handle;
    // p_msg->req.write_req.gattc_params.len      = length;
    // p_msg->req.write_req.gattc_params.p_value  = (uint8_t *)p_data;
    // p_msg->req.write_req.gattc_params.flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    // // p_msg->req.write_req.gattc_params.offset   = 0;
    // p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    // // p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    // // p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    // p_msg->conn_handle                         = p_imu_service->conn_handle;
    // p_msg->type                                = WRITE_REQ;

    // tx_buffer_process();
    // return NRF_SUCCESS;
}


// uint32_t ble_nus_c_string_send(ble_nus_c_t * p_ble_nus_c, uint8_t * p_string, uint16_t length)
// {
//     VERIFY_PARAM_NOT_NULL(p_ble_nus_c);

//     nrf_ble_gq_req_t write_req;

//     memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

//     if (length > BLE_NUS_MAX_DATA_LEN)
//     {
//         NRF_LOG_WARNING("Content too long.");
//         return NRF_ERROR_INVALID_PARAM;
//     }
//     if (p_ble_nus_c->conn_handle == BLE_CONN_HANDLE_INVALID)
//     {
//         NRF_LOG_WARNING("Connection handle invalid.");
//         return NRF_ERROR_INVALID_STATE;
//     }

//     write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
//     write_req.error_handler.cb            = gatt_error_handler;
//     write_req.error_handler.p_ctx         = p_ble_nus_c;
//     write_req.params.gattc_write.handle   = p_ble_nus_c->handles.nus_rx_handle;
//     write_req.params.gattc_write.len      = length;
//     write_req.params.gattc_write.offset   = 0;
//     write_req.params.gattc_write.p_value  = p_string;
//     write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_CMD;
//     write_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

//     return nrf_ble_gq_item_add(p_ble_nus_c->p_gatt_queue, &write_req, p_ble_nus_c->conn_handle);
// }


uint32_t ble_imu_service_c_handles_assign(ble_imu_service_c_t    * p_ble_imu_service_c,
                                  uint16_t         conn_handle,
                                  const imu_service_db_t * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_imu_service_c);

    p_ble_imu_service_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_imu_service_c->peer_imu_service_db = *p_peer_handles;
    }

    NRF_LOG_INFO("conn_handle assign: %d", p_ble_imu_service_c->conn_handle);

    // return NRF_SUCCESS;

    return nrf_ble_gq_conn_handle_register(p_ble_imu_service_c->p_gatt_queue, conn_handle);
}

// #endif // NRF_MODULE_ENABLED(BLE_IMU_SERVICE_C)
