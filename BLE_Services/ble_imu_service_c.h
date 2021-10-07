/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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
/**@file
 *
 * @defgroup ble_imu_service_c Thingy Enviroment Service Client
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    The Thingy Enviroment Service client can be used to set a LED, and read a button state on a
 *           Thingy Enviroment service server.
 *
 * @details  This module contains the APIs and types exposed by the Thingy Enviroment Service Client
 *           module. These APIs and types can be used by the application to perform discovery of
 *           Thingy Enviroment Service at the peer and interact with it.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_imu_service_c_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_imu_service_C_BLE_OBSERVER_PRIO,
 *                                   ble_imu_service_c_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef BLE_IMU_SERVICE_C_H__
#define BLE_IMU_SERVICE_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif


/**@brief   Macro for defining a ble_imu_service_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_IMU_SERVICE_C_DEF(_name)                                                                        \
static ble_imu_service_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_IMU_SERVICE_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_imu_service_c_on_ble_evt, &_name)

/**@brief   Macro for defining multiple ble_imu_service_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 */
#define BLE_IMU_SERVICE_C_ARRAY_DEF(_name, _cnt)                                                            \
static ble_imu_service_c_t _name[_cnt];                                                                     \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                                                                \
                      BLE_IMU_SERVICE_C_BLE_OBSERVER_PRIO,                                                  \
                      ble_imu_service_c_on_ble_evt, &_name, _cnt)


#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_IMU_SERVICE_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_IMU_SERVICE_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif



// #define THINGY_UUID_BASE        {0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x00, 0x68, 0xEF}
#define THINGY_UUID_BASE                  {0x5d, 0x82, 0x7d, 0x69, 0x83, 0xd1, 0x7c, 0x96, 0x2e, 0x43, 0xfb, 0x95, 0x54, 0x03, 0xaa, 0xcb}


#define THINGY_UIS_UUID_SERVICE     0x0300
#define THINGY_UIS_UUID_BUTTON_CHAR 0x0302
#define THINGY_UIS_UUID_LED_CHAR    0x0301

#define IMU_SERVICE_UUID_SERVICE          0x0400
#define IMU_SERVICE_UUID_CONFIG_CHAR      0x0401                      /**< The UUID of the config Characteristic. */
#define IMU_SERVICE_UUID_TAP_CHAR         0x0402                      /**< The UUID of the tap Characteristic. */
#define IMU_SERVICE_UUID_ADC_CHAR 0x0403                      /**< The UUID of the adc Characteristic. */
#define IMU_SERVICE_UUID_QUATERNION_CHAR  0x0404                      /**< The UUID of the quaternion Characteristic. */
#define IMU_SERVICE_UUID_PEDOMETER_CHAR   0x0405                      /**< The UUID of the pedometer Characteristic. */
#define IMU_SERVICE_UUID_RAW_CHAR         0x0406                      /**< The UUID of the raw data Characteristic. */
#define IMU_SERVICE_UUID_EULER_CHAR       0x0407                      /**< The UUID of the euler Characteristic. */
#define IMU_SERVICE_UUID_ROT_MAT_CHAR     0x0408                      /**< The UUID of the rotation matrix Characteristic. */
#define IMU_SERVICE_UUID_HEADING_CHAR     0x0409                      /**< The UUID of the compass heading Characteristic. */
#define IMU_SERVICE_UUID_GRAVITY_CHAR     0x040A                      /**< The UUID of the gravity vector Characteristic. */

// How many packets (QUAT - RAW) are grouped in a message
#define BLE_PACKET_BUFFER_COUNT     5

/**@brief TES Client event type. */
typedef enum
{
    BLE_IMU_SERVICE_C_EVT_DISCOVERY_COMPLETE = 1,       /**< Event indicating that the Thingy enviroment Service has been discovered at the peer. */
    BLE_IMU_SERVICE_EVT_CONFIG_RECEIVED,
    BLE_IMU_SERVICE_EVT_TAP,
    BLE_IMU_SERVICE_EVT_ADC,
    BLE_IMU_SERVICE_EVT_QUAT,
    BLE_IMU_SERVICE_EVT_PEDOMETER,
    BLE_IMU_SERVICE_EVT_RAW,
    BLE_IMU_SERVICE_EVT_EULER,
    BLE_IMU_SERVICE_EVT_ROT_MAT,
    BLE_IMU_SERVICE_EVT_HEADING,
    BLE_IMU_SERVICE_EVT_GRAVITY,     /**< Event indicating that a notification of the Thingy enviroment temperature characteristic has been received from the peer. */
} ble_imu_service_c_evt_type_t;


typedef struct
{
    uint32_t raw[40];
}ble_imu_service_adc_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ble_imu_service_raw_accel_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ble_imu_service_raw_gyro_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ble_imu_service_raw_compass_t;

typedef struct
{
    ble_imu_service_raw_accel_t   accel;
    ble_imu_service_raw_gyro_t    gyro;
    ble_imu_service_raw_compass_t compass;
} ble_imu_service_single_raw_t;

typedef struct
{
    ble_imu_service_single_raw_t single_raw[BLE_PACKET_BUFFER_COUNT];
} ble_imu_service_raw_t;

typedef  struct
{
    int32_t w;
    int32_t x;
    int32_t y;
    int32_t z;
} ble_imu_service_single_quat_t;

typedef struct
{
    ble_imu_service_single_quat_t quat[BLE_PACKET_BUFFER_COUNT];
} ble_imu_service_quat_t;

typedef struct
{
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
} ble_imu_service_euler_t;


typedef struct
{
    int8_t  integer;
    uint8_t decimal;
} ble_imu_service_temperature_t;

typedef struct
{
    int32_t  integer;
    uint8_t  decimal;
} ble_imu_service_pressure_t;

typedef uint8_t ble_imu_service_humidity_t;

typedef struct
{
    uint16_t eco2_ppm; ///< The equivalent CO2 (eCO2) value in parts per million.
    uint16_t tvoc_ppb; ///< The Total Volatile Organic Compound (TVOC) value in parts per billion.
} ble_imu_service_gas_t;

typedef struct
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
} ble_imu_service_color_t;

/**@brief Structure containing the config value received from the peer. */
typedef struct
{
    bool                      gyro_enabled;
    bool                      accel_enabled;
    bool                      mag_enabled;
    bool                      euler_enabled;
    bool                      quat6_enabled;
    bool                      quat9_enabled;
    uint16_t                  motion_freq_hz;
    bool                      wom_enabled;
    bool                      sync_enabled;
    uint64_t                  sync_start_time;
    bool                      stop;
    bool                      adc_enabled;
} ble_imu_service_config_t;

/**@brief Structure containing the event value received from the peer. */
typedef union
{
    ble_imu_service_quat_t quat_data;
    ble_imu_service_euler_t euler_data;
    ble_imu_service_raw_t raw_data;
    ble_imu_service_adc_t adc_data;
} ble_evt_value_t;

/**@brief Structure containing the handles related to the Thingy Enviroment Service found on the peer. */
typedef struct
{
    uint16_t config_cccd_handle;
    uint16_t euler_cccd_handle;       /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t heading_cccd_handle;          /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t adc_cccd_handle;          /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t pedometer_cccd_handle;               /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t quat_cccd_handle;             /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t raw_cccd_handle;            /**< Handle of the CCCD of the <...> characteristic. */
    uint16_t rot_cccd_handle;       /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t tap_cccd_handle;          /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t gravity_cccd_handle;          /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t config_handle;
    uint16_t euler_handle;               /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t heading_handle;             /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t adc_handle;            /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t pedometer_handle;               /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t quat_handle;             /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t raw_handle;            /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t rot_handle;            /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t tap_handle;               /**< Handle of the <...> characteristic as provided by the SoftDevice. */
    uint16_t gravity_handle;             /**< Handle of the <...> characteristic as provided by the SoftDevice. */
} imu_service_db_t;

/**@brief Thingy enviroment Event structure. */
typedef struct
{
    ble_imu_service_c_evt_type_t evt_type;    /**< Type of the event. */
    uint16_t             conn_handle; /**< Connection handle on which the event occured.*/
    union
    {
        ble_evt_value_t value;          /**< The value of the event received. This will be filled if the evt_type is @ref BLE_IMU_SERVICE_C_EVT_<...>_NOTIFICATION. */
        imu_service_db_t     peer_db;         /**< Thingy enviroment service related handles found on the peer device. This will be filled if the evt_type is @ref BLE_imu_service_C_EVT_DISCOVERY_COMPLETE.*/
    } params;
} ble_imu_service_c_evt_t;

// Forward declaration of the ble_imu_service_c_t type.
typedef struct ble_imu_service_c_s ble_imu_service_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_imu_service_c_evt_handler_t) (ble_imu_service_c_t * p_ble_imu_service_c, ble_imu_service_c_evt_t * p_evt);

/**@brief Thingy Enviroment Client structure. */
struct ble_imu_service_c_s
{
    uint16_t                conn_handle;  /**< Connection handle as provided by the SoftDevice. */
    imu_service_db_t                peer_imu_service_db;  /**< Handles related to tes on the peer*/
    ble_imu_service_c_evt_handler_t evt_handler;  /**< Application event handler to be called when there is an event related to the Thingy Enviroment service. */
    uint8_t                 uuid_type;    /**< UUID type. */
    // ble_gatts_char_handles_t config_handles;               /**< Handles related to the config characteristic (as provided by the S132 SoftDevice). */
    nrf_ble_gq_t            * p_gatt_queue; /**< Pointer to the BLE GATT Queue instance. */
};

/**@brief Thingy Enviroment Client initialization structure. */
typedef struct
{
    ble_imu_service_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Thingy Enviroment Client module whenever there is an event related to the Thingy Enviroment Service. */
    nrf_ble_gq_t            * p_gatt_queue; /**< Pointer to the BLE GATT Queue instance. */
} ble_imu_service_c_init_t;


/**@brief Function for initializing the Thingy Enviroment client module.
 *
 * @details This function will register with the DB Discovery module. There it registers for the
 *          Thingy Enviroment Service. Doing so will make the DB Discovery module look for the presence
 *          of a Thingy Enviroment Service instance at the peer when a discovery is started.
 *
 * @param[in] p_ble_imu_service_c      Pointer to the Thingy Enviroment client structure.
 * @param[in] p_ble_imu_service_c_init Pointer to the Thingy Enviroment initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. Otherwise an error code. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_imu_service_c_init(ble_imu_service_c_t * p_ble_imu_service_c, ble_imu_service_c_init_t * p_ble_imu_service_c_init);


/**@brief Function for handling BLE events from the SoftDevice.
 *
 * @details This function will handle the BLE events received from the SoftDevice. If a BLE event
 *          is relevant to the Thingy Enviroment Client module, then it uses it to update interval
 *          variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the Thingy Enviroment client structure.
 */
void ble_imu_service_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for requesting the peer to start sending notification of the different characteristics
 *        Characteristic.
 *
 * @details This function will enable to notification of the characteristic at the peer
 *          by writing to the CCCD of the temperature Characteristic.
 *
 * @param[in] p_ble_imu_service_c Pointer to the Thingy Enviroment Client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 *          NRF_ERROR_INVALID_STATE if no connection handle has been assigned (@ref ble_imu_service_c_handles_assign)
 *          NRF_ERROR_NULL if the given parameter is NULL
 */
uint32_t ble_imu_service_c_quaternion_notif_enable(ble_imu_service_c_t * p_ble_imu_service_c);
uint32_t ble_imu_service_c_adc_notif_enable(ble_imu_service_c_t * p_ble_imu_service_c);
uint32_t ble_imu_service_c_euler_notif_enable(ble_imu_service_c_t * p_ble_imu_service_c);
uint32_t ble_imu_service_c_raw_notif_enable(ble_imu_service_c_t * p_ble_imu_service_c);

uint32_t ble_imu_service_config_set(ble_imu_service_c_t * p_imu_service, ble_imu_service_config_t * p_data);


/**@brief Function for handling events from the database discovery module.
 *
 * @details Call this function when getting a callback event from the DB discovery module. This
 *          function will handle an event from the database discovery module, and determine if it
 *          relates to the discovery of Thingy Enviroment service at the peer. If so, it will call the
 *          application's event handler indicating that the Thingy Enviroment service has been discovered
 *          at the peer. It also populates the event with the service related information before
 *          providing it to the application.
 *
 * @param[in] p_ble_imu_service_c Pointer to the Thingy Enviroment client structure.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 */
void ble_imu_service_on_db_disc_evt(ble_imu_service_c_t * p_ble_imu_service_c, ble_db_discovery_evt_t const * p_evt);


/**@brief     Function for assigning a Handles to this instance of imu_service_c.
 *
 * @details Call this function when a link has been established with a peer to associate this link
 *          to this instance of the module. This makes it  possible to handle several links and
 *          associate each link to a particular instance of this module.
 *
 * @param[in] p_ble_imu_service_c    Pointer to the Thingy Enviroment client structure instance to associate.
 * @param[in] conn_handle    Connection handle to associate with the given Thingy Enviroment Client Instance.
 * @param[in] p_peer_handles Thingy Enviroment Service handles found on the peer (from @ref BLE_imu_service_C_EVT_DISCOVERY_COMPLETE event).
 *
 */
uint32_t ble_imu_service_c_handles_assign(ble_imu_service_c_t *    p_ble_imu_service_c,
                                  uint16_t         conn_handle,
                                  const imu_service_db_t * p_peer_handles);


#ifdef __cplusplus
}
#endif

#endif // BLE_imu_service_C_H__

/** @} */
