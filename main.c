/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
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

// UARTE using easyDMA
#include "nrf_drv_uart.h"
#include "app_fifo.h"

// Application scheduler
#include "app_scheduler.h"

// IMU Params
#include "imu_params.h"

// Receive data from Thingy motion service
#include "ble_tes_c.h"



// Create a FIFO structure
typedef struct buffer
{
    app_fifo_t uart_dma_difo;
    app_fifo_t received_data_fifo;  
    uint8_t uart_dma_tx_buff[100];
}BUFFER;

// Initialisation of struct to keep track of different buffers
BUFFER buffer;


typedef struct imu
{
    bool gyro_enabled;
    bool accel_enabled;
    bool mag_enabled;
    bool quat6_enabled;
    bool quat9_enabled;
    bool euler_enabled;
    uint32_t period; // period in milliseconds (ms)
    uint16_t packet_length;
    int received_packet_counter;
    nrf_drv_uart_t uart;
    uint32_t evt_scheduled;
} IMU;

// Initialisation of IMU struct
IMU imu = {
    .received_packet_counter = 0,
    .evt_scheduled = 0,
    .uart = NRF_DRV_UART_INSTANCE(0),
};


// APP Scheduler
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 200                                      /**< Maximum number of events in the scheduler queue. */

#define SYNC_FREQ 2 // Hz

/////////////// LED BUTTON BLINK /////////////////////
#define SPARKFUN_LED 7
#define SPARKFUN_BUTTON 6

#define PIN_IN BUTTON_1
#define PIN_OUT LED_1
/////////////// LED BUTTON BLINK /////////////////////

#define APP_BLE_CONN_CFG_TAG 1  /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO 3 /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE 2048 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 2048 /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA 0 //1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

/* CHANGES */
//BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
BLE_NUS_C_ARRAY_DEF(m_ble_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< BLE Nordic UART Service (NUS) client instances. */
/* END CHANGES */
BLE_TES_C_ARRAY_DEF(m_thingy_tes_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);                  /**< Structure used to identify the battery service. */



NRF_BLE_GATT_DEF(m_gatt);        /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc); /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);        /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue, /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
//static uint16_t m_ble_nus_max_data_len = 247 - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

// Event handler DIY
void data_evt_sceduled(void *p_event_data, uint16_t event_size)
{
    nrf_gpio_pin_set(22);
    uint32_t err_code;

    while (imu.evt_scheduled > 0)
    {
        NRF_LOG_INFO("App scheduler execute: %d", imu.evt_scheduled);
        NRF_LOG_FLUSH();

        uint32_t classification_byte_len = 1;
        uint8_t classification_byte[classification_byte_len];

        bool read_success = false;
        uint32_t temp_len;
        float quat[4];
        float other[3];
        char string_send[200];

        // Get 1 byte from buffer to determine which sort of data we are dealing with
        if (app_fifo_read(&buffer.received_data_fifo, classification_byte, &classification_byte_len) == NRF_SUCCESS)
        {
            switch (classification_byte[0])
            {
            case ENABLE_QUAT6:
                // Get 4 floats
                temp_len = 4 * sizeof(float);
                // NRF_LOG_INFO("temp_len: %d", temp_len);
                if (app_fifo_read(&buffer.received_data_fifo, (uint8_t *)quat, &temp_len) == NRF_SUCCESS)
                {
                    NRF_LOG_INFO("Read QUAT6");
                    NRF_LOG_INFO("%d %d %d %d", 1000*quat[0], 1000*quat[1], 1000*quat[2], 1000*quat[3]);
                    read_success = true;
                    sprintf(string_send, "w%.2fwa%.2fab%.2fbc%.2fc\n", quat[0], quat[1], quat[2], quat[3]);
                    NRF_LOG_INFO("%s", string_send);
                }
                break;
            case ENABLE_QUAT9:
                // Get 4 floats
                temp_len = 4 * sizeof(float);
                // NRF_LOG_INFO("temp_len: %d", temp_len);
                if (app_fifo_read(&buffer.received_data_fifo, (uint8_t *)quat, &temp_len) == NRF_SUCCESS)
                {
                    NRF_LOG_INFO("Read QUAT9");
                    NRF_LOG_INFO("%d %d %d %d", 1000*quat[0], 1000*quat[1], 1000*quat[2], 1000*quat[3]);
                    read_success = true;
                    sprintf(string_send, "w%fwa%fab%fbc%fc\n", quat[0], quat[1], quat[2], quat[3]);
                    NRF_LOG_INFO("%s", string_send);
                }
                break;
            case ENABLE_EULER:
                // Get 3 floats
                temp_len = 3 * sizeof(float);
                if (app_fifo_read(&buffer.received_data_fifo, (uint8_t *)other, &temp_len) == NRF_SUCCESS)
                {
                    NRF_LOG_INFO("Read EULER");
                    read_success = true;
                    // Convert to string
                    sprintf(string_send, "Euler: x: %f	y: %f	z: %f\n", other[0], other[1], other[2]);
                }
                break;
            case ENABLE_GYRO:
                // Get 3 floats
                temp_len = 3 * sizeof(float);
                if (app_fifo_read(&buffer.received_data_fifo, (uint8_t *)other, &temp_len) == NRF_SUCCESS)
                {
                    NRF_LOG_INFO("Read GYRO");
                    read_success = true;
                    // Convert to string
                    sprintf(string_send, "Gyro: x: %.2f	y: %.2f	z: %.2f\n", other[0], other[1], other[2]);
                }
                break;
            case ENABLE_ACCEL:
                // Get 3 floats
                temp_len = 3 * sizeof(float);
                if (app_fifo_read(&buffer.received_data_fifo, (uint8_t *)other, &temp_len) == NRF_SUCCESS)
                {
                    NRF_LOG_INFO("Read ACC");
                    read_success = true;
                    // Convert to string
                    sprintf(string_send, "Acc: x: %.2f	y: %.2f	z: %.2f\n", other[0], other[1], other[2]);
                }
                break;
            case ENABLE_MAG:
                // Get 3 floats
                temp_len = 3 * sizeof(float);
                if (app_fifo_read(&buffer.received_data_fifo, (uint8_t *)other, &temp_len) == NRF_SUCCESS)
                {
                    NRF_LOG_INFO("Read MAG");
                    read_success = true;
                    // Convert to string
                    sprintf(string_send, "Mag: x: %f	y: %f	z: %f\n", other[0], other[1], other[2]);
                }
                break;
            default:
                // in case of error
                NRF_LOG_INFO("Error detecting classification_byte: %X", classification_byte[0]);
                break;
            }
        }

        // Get data from FIFO buffer if data is correctly recognized
        if (read_success)
        {

            // NRF_LOG_INFO("read_success");
            // NRF_LOG_FLUSH();

            uint32_t string_len = 0;
            do
            {
                string_len++;
            } while (string_send[string_len] != '\n');
            string_len++;

            // Put the data in FIFO buffer
            err_code = app_fifo_write(&buffer.uart_dma_difo, (uint8_t *)string_send, &string_len);
            if (err_code == NRF_ERROR_NO_MEM)
            {
                NRF_LOG_INFO("UART FIFO BUFFER FULL!");
            }

            if (err_code == NRF_SUCCESS)
            {
                //								NRF_LOG_INFO("Data in FIFO");
                // The new byte has been added to FIFO. It will be picked up from there
                // (in 'uart_event_handler') when all preceding bytes are transmitted.
                // But if UART is not transmitting anything at the moment, we must start
                // a new transmission here.
                if (!nrf_drv_uart_tx_in_progress(&imu.uart))
                {
                    //										NRF_LOG_INFO("TX not in progress");
                    // This operation should be almost always successful, since we've
                    // just added a byte to FIFO, but if some bigger delay occurred
                    // (some heavy interrupt handler routine has been executed) since
                    // that time, FIFO might be empty already.
                    if (app_fifo_read(&buffer.uart_dma_difo, buffer.uart_dma_tx_buff, &string_len) == NRF_SUCCESS)
                    {
                        //												NRF_LOG_INFO("FIFO read");
                        do
                        {
                            err_code = nrf_drv_uart_tx(&imu.uart, buffer.uart_dma_tx_buff, (uint8_t)string_len);
                            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                            {
                                NRF_LOG_ERROR("nrf_drv_uart_tx failed");
                                APP_ERROR_CHECK(err_code);
                            }
                        } while (err_code == NRF_ERROR_BUSY);
                        //												NRF_LOG_INFO("UART TX OK");
                    }
                }
            }
            imu.evt_scheduled--; // Needs to be here: problem: can skip some malformed packets - otherwise the app_scheduler will continue to execute and block the cpu
        }
        
    }
    nrf_gpio_pin_clear(22);
}

/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
    {
        .uuid = BLE_UUID_NUS_SERVICE,
        .type = NUS_SERVICE_UUID_TYPE};

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const *p_scan_evt)
{
    ret_code_t err_code;

    switch (p_scan_evt->scan_evt_id)
    {
    case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
    {
        err_code = p_scan_evt->params.connecting_err.err_code;
        APP_ERROR_CHECK(err_code);
    }
    break;

    case NRF_BLE_SCAN_EVT_CONNECTED:
    {
        ble_gap_evt_connected_t const *p_connected =
            p_scan_evt->params.connected.p_connected;
        // Scan is automatically stopped by the connection.
        NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                     p_connected->peer_addr.addr[0],
                     p_connected->peer_addr.addr[1],
                     p_connected->peer_addr.addr[2],
                     p_connected->peer_addr.addr[3],
                     p_connected->peer_addr.addr[4],
                     p_connected->peer_addr.addr[5]);
    }
    break;

    case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
    {
        NRF_LOG_INFO("Scan timed out.");
        scan_start();
    }
    break;

    default:
        break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t *p_evt)
{
    /* CHANGES */
    //ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);
    /* END CHANGES */

    // Add discovery for TMS service
    ble_thingy_tes_on_db_disc_evt(&m_thingy_tes_c[p_evt->conn_handle], p_evt);

}

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t *p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            //					NRF_LOG_INFO("Test");
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len - 1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY)
            ;
    }

    /* CHANGES
		
		if (ECHOBACK_BLE_UART_DATA)
    {
         //Send data back to the peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
		END CHANGES */

    if (ECHOBACK_BLE_UART_DATA)
    {
        for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
        {
            // Send data back to the peripheral.
            do
            {
                ret_val = ble_nus_c_string_send(&m_ble_nus_c[c], p_data, data_len);
                if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY) && (ret_val != NRF_ERROR_INVALID_STATE))
                {
                    NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                    APP_ERROR_CHECK(ret_val);
                }
            } while (ret_val == NRF_ERROR_BUSY);
        }
    }

    /* END CHANGES */
}

// Put every received byte in a FIFO buffer
static void ble_nus_data_received_uart_print(uint8_t *p_data, uint16_t data_len)
{
    nrf_gpio_pin_set(18);

    NRF_LOG_INFO("%X", p_data[0]);

    ret_code_t err_code;

    uint8_t temp[data_len / sizeof(uint8_t)];

    // Copy data to quaternion varaiable
    memcpy(temp, p_data, data_len);

    uint32_t temp_len = sizeof(temp);

    // Put the received data in FIFO buffer
    err_code = app_fifo_write(&buffer.received_data_fifo, temp, &temp_len);
    if (err_code == NRF_ERROR_NO_MEM)
    {
        NRF_LOG_INFO("RECEIVED DATA FIFO BUFFER FULL!");
    }
    if (err_code == NRF_SUCCESS)
    {
        // Signal to event handler to execute sprintf + start UART transmission
        // If there are already events in the queue
        if (imu.evt_scheduled > 0)
        {
            imu.evt_scheduled++;
        }
        // If there are not yet any events in the queue, schedule event. In gpiote_evt_sceduled all callbacks are called
        else
        {
            imu.evt_scheduled++;
            err_code = app_sched_event_put(0, 0, data_evt_sceduled);
            APP_ERROR_CHECK(err_code);
        }
    }

    nrf_gpio_pin_clear(18);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t *p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
    /**@snippet [Handling data from UART] */
    case APP_UART_DATA_READY:
        //            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
        //            index++;

        //            if ((data_array[index - 1] == '\n') ||
        //                (data_array[index - 1] == '\r') ||
        //                (index >= (m_ble_nus_max_data_len)))
        //            {
        //                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
        //                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

        //
        //							/* CHANGES
        //                do
        //                {
        //                    ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
        //                    if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
        //                    {
        //                        APP_ERROR_CHECK(ret_val);
        //                    }
        //                } while (ret_val == NRF_ERROR_RESOURCES);
        //							END CHANGES	*/
        //
        //							for(int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
        //                {
        //                    do
        //                    {
        //                        ret_val = ble_nus_c_string_send(&m_ble_nus_c[c], data_array, index);
        //                        if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
        //                        {
        //                            APP_ERROR_CHECK(ret_val);
        //                        }
        //                    } while (ret_val == NRF_ERROR_RESOURCES);
        //                }
        //							/* END CHANGES */
        //
        //                index = 0;
        //            }
        break;

    /**@snippet [Handling data from UART] */
    case APP_UART_COMMUNICATION_ERROR:
        NRF_LOG_ERROR("Communication error occurred while handling UART.");
        NRF_LOG_INFO("Communication error occurred while handling UART.");
        NRF_LOG_FLUSH();
        APP_ERROR_HANDLER(p_event->data.error_communication);
        break;

    case APP_UART_FIFO_ERROR:
        NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
        APP_ERROR_HANDLER(p_event->data.error_code);
        break;

    default:
        break;
    }
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t *p_ble_nus_c, ble_nus_c_evt_t const *p_ble_nus_evt)
{
    ret_code_t err_code;

    NRF_LOG_INFO("ble_nus_c_evt_handler");

    switch (p_ble_nus_evt->evt_type)
    {
    case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
        NRF_LOG_INFO("Discovery complete.");
        err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
        APP_ERROR_CHECK(err_code);

        err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Connected to device with Nordic UART Service.");
        break;

    case BLE_NUS_C_EVT_NUS_TX_EVT:
        //            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
        ble_nus_data_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
        // NRF_LOG_INFO("Character received");
        // NRF_LOG_INFO("Receive counter:	%d", imu.received_packet_counter);
        imu.received_packet_counter++;
        break;

    case BLE_NUS_C_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected.");
        scan_start();
        break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */

/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
    case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
        // Prepare wakeup buttons.
        err_code = bsp_btn_ble_sleep_mode_prepare();
        APP_ERROR_CHECK(err_code);
        break;

    default:
        break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    ret_code_t err_code;
    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        /* CHANGES */
        //err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
        err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_ble_evt->evt.gap_evt.conn_handle], p_ble_evt->evt.gap_evt.conn_handle, NULL);
        /* END CHANGES */

        APP_ERROR_CHECK(err_code);

        // TMS CHANGES - add handles
        err_code = ble_tes_c_handles_assign(&m_thingy_tes_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
        APP_ERROR_CHECK(err_code);

        /* ADDED CHANGES*/
        if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
        {
            // Resume scanning.
            scan_start();
        }
        /* END ADDED CHANGES */

        err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);

        // start discovery of services. The NUS Client waits for a discovery result
        err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
        APP_ERROR_CHECK(err_code);

        // Change to 2MBIT PHY
        NRF_LOG_DEBUG("PHY update!");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_2MBPS,
                .tx_phys = BLE_GAP_PHY_2MBPS,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);

        break;

    case BLE_GAP_EVT_DISCONNECTED:

        NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                     p_gap_evt->conn_handle,
                     p_gap_evt->params.disconnected.reason);
        break;

    case BLE_GAP_EVT_TIMEOUT:
        if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
        {
            NRF_LOG_INFO("Connection Request timed out.");
        }
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported.
        err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        // Accepting parameters requested by peer.
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                &p_gap_evt->params.conn_param_update_request.conn_params);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

void config_imu(uint8_t *config, uint8_t len)
{
    uint32_t err_code;

    // Remote adjustment of settings of IMU
    // Send data back to the peripheral.
    uint8_t p_data[len];
    memcpy(p_data, config, len);

    for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
    {
        do
        {
            err_code = ble_nus_c_string_send(&m_ble_nus_c[c], p_data, len);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY) && (err_code != NRF_ERROR_INVALID_STATE))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", err_code);
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_BUSY);
    }
    NRF_LOG_INFO("CONFIG SEND!");
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
    // TimeSync begin
    case BSP_EVENT_KEY_0:
    {
        static bool m_send_sync_pkt = false;
        if (m_send_sync_pkt)
        {
            m_send_sync_pkt = false;
            bsp_board_leds_off();
            err_code = ts_tx_stop();
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");
        }
        else
        {
            m_send_sync_pkt = true;
            bsp_board_leds_on();
            err_code = ts_tx_start(SYNC_FREQ); // Frequency of synchronization packets
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
        }
    }
    break;
        // TimeSync end

    case BSP_EVENT_KEY_1:
    {
        uint8_t temp_config1[] = {ENABLE_GYRO, ENABLE_ACCEL, ENABLE_QUAT6};
        config_imu(temp_config1, sizeof(temp_config1));
        break;
    }
    case BSP_EVENT_KEY_2:
    {
        uint8_t temp_config3[] = {ENABLE_GYRO, ENABLE_ACCEL};
        config_imu(temp_config3, sizeof(temp_config3));
        break;
    }
    case BSP_EVENT_KEY_3:
    {
        uint8_t temp_config2[] = {ENABLE_QUAT6};
        config_imu(temp_config2, sizeof(temp_config2));
        break;
    }
    case BSP_EVENT_SLEEP:
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        break;

    case BSP_EVENT_DISCONNECT:

        /* CHANGES
						err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
				END CHANGES */

        for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
        {
            err_code = sd_ble_gap_disconnect(m_ble_nus_c[c].conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
        /* END CHANGES */

        break;

    default:
        break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
        {
            .rx_pin_no = RX_PIN_NUMBER, //26,//RX_PIN_NUMBER,//26
            .tx_pin_no = TX_PIN_NUMBER, //27,//TX_PIN_NUMBER,//27
            .rts_pin_no = RTS_PIN_NUMBER,
            .cts_pin_no = CTS_PIN_NUMBER,
            .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
            .use_parity = false,
            .baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200};

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue = &m_ble_gatt_queue;

    /* CHANGES
    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
	END CHANGES */
    for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
    {
        err_code = ble_nus_c_init(&m_ble_nus_c[c], &init);
        APP_ERROR_CHECK(err_code);
    }
    /* END CHANGES */
}

/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    // err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);

    // TimeSync begin
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    // TimeSync end

    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void sync_timer_init(void)
{
    uint32_t err_code;
    uint8_t rf_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x19};
    ts_params_t ts_params;
    // Debug pin:
    // nRF52-DK (PCA10040) Toggle P0.24 from sync timer to allow pin measurement
    // nRF52840-DK (PCA10056) Toggle P1.14 from sync timer to allow pin measurement
#if defined(BOARD_PCA10040)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, 24), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#elif defined(BOARD_PCA10056)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 14), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#else
#warning Debug pin not set
#endif
    nrf_ppi_channel_endpoint_setup(
        NRF_PPI_CHANNEL0,
        (uint32_t)nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE4),
        nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);
    ts_params.high_freq_timer[0] = NRF_TIMER3;
    ts_params.high_freq_timer[1] = NRF_TIMER2;
    ts_params.rtc = NRF_RTC1;
    ts_params.egu = NRF_EGU3;
    ts_params.egu_irq_type = SWI3_EGU3_IRQn;
    ts_params.ppi_chg = 0;
    ts_params.ppi_chns[0] = 1;
    ts_params.ppi_chns[1] = 2;
    ts_params.ppi_chns[2] = 3;
    ts_params.ppi_chns[3] = 4;
    ts_params.rf_chn = 125; /* For testing purposes */
    memcpy(ts_params.rf_addr, rf_address, sizeof(rf_address));
    err_code = ts_init(&ts_params);
    APP_ERROR_CHECK(err_code);
    err_code = ts_enable();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Started listening for beacons.\r\n");
    NRF_LOG_INFO("Press Button 1 to start sending sync beacons\r\n");
}

/////////////// LED BUTTON BLINK /////////////////////
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_drv_gpiote_out_toggle(PIN_OUT);

    /* CHANGES */

    NRF_LOG_INFO("BUTTON PRESS");

    /* END CHANGES */
}
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}
/////////////// LED BUTTON BLINK /////////////////////

char const *phy_str(ble_gap_phys_t phys)
{
    static char const *str[] =
        {
            "1 Mbps",
            "2 Mbps",
            "Coded",
            "Unknown"};

    switch (phys.tx_phys)
    {
    case BLE_GAP_PHY_1MBPS:
        return str[0];

    case BLE_GAP_PHY_2MBPS:
    case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS:
    case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED:
        return str[1];

    case BLE_GAP_PHY_CODED:
        return str[2];

    default:
        return str[3];
    }
}

//void current_config_print(nrf_cli_t const * p_cli)
//{
//    NRF_LOG_INFO(p_cli, "==== Current test configuration ====\r\n");
//    NRF_LOG_INFO(p_cli,
//                    "ATT MTU size:\t\t%d\r\n"
//                    "Data length:\t\t%d\r\n"
//                    "Connection interval:\t%d units\r\n"
//                    "Connection length ext:\t%s\r\n"
//                    "Preferred PHY:\t\t%s\r\n",
//                    params.att_mtu,
//                    params.data_len,
//                    params.conn_interval,
//                    params.conn_evt_len_ext_enabled ? "on" : "off",
//                    phy_str(params.phys));
//    NRF_LOG_INFO(p_cli, "GAP event length:\t%d\r\n",
//                    NRF_SDH_BLE_GAP_EVENT_LENGTH);
//}

void conn_evt_len_ext_set(void)
{
    ret_code_t err_code;
    ble_opt_t opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = 1;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}

static void usr_uarte_evt_handler(nrf_drv_uart_event_t *p_event, void *p_context)
{
    uint32_t err_code;

    switch (p_event->type)
    {
    case NRF_DRV_UART_EVT_TX_DONE: ///< Requested TX transfer completed.
    {
        // TODO send data more efficiently
        uint32_t index = 255;
        // Get next bytes from FIFO.
        if (app_fifo_read(&buffer.uart_dma_difo, buffer.uart_dma_tx_buff, &index) == NRF_SUCCESS)
        {
            nrf_drv_uart_tx(&imu.uart, buffer.uart_dma_tx_buff, (uint8_t)index);
            //								NRF_LOG_INFO("index evt %d", index);
            //								NRF_LOG_INFO("Send next byte from evt handler");
        }
        else
        {
                // Last byte from FIFO transmitted, notify the application.
                // app_uart_event.evt_type = APP_UART_TX_EMPTY;
                // m_event_handler(&app_uart_event);
                NRF_LOG_INFO("UART TX EMPTY");
        }
        NRF_LOG_INFO("UART TX done");
        break;
    }
    case NRF_DRV_UART_EVT_RX_DONE: ///< Requested RX transfer completed.
        break;
    case NRF_DRV_UART_EVT_ERROR: ///< Error reported by UART peripheral.
        NRF_LOG_INFO("Error in NRF_DRV_UART_EVT_ERROR");
        break;
    default:
        break;
    }
}

void uart_dma_init()
{
    ret_code_t err_code;

    nrf_drv_uart_config_t const uart_config_params = {
        .pseltxd = TX_PIN_NUMBER,
        .pselrxd = RX_PIN_NUMBER,
        .pselcts = CTS_PIN_NUMBER,
        .pselrts = RTS_PIN_NUMBER,
        .p_context = NULL,
        .hwfc = (nrf_uart_hwfc_t)UART_DEFAULT_CONFIG_HWFC,
        .parity = (nrf_uart_parity_t)UART_DEFAULT_CONFIG_PARITY,
        .baudrate = (nrf_uart_baudrate_t)UART_BAUDRATE_BAUDRATE_Baud1M,
        .interrupt_priority = UART_DEFAULT_CONFIG_IRQ_PRIORITY, //NRF_DRV_UART_DEFAULT_CONFIG_USE_EASY_DMA                                 \ // Yes, use easyDMA!
    };

    err_code = nrf_drv_uart_init(&imu.uart, &uart_config_params, usr_uarte_evt_handler);

    APP_ERROR_CHECK(err_code);
}



void set_imu_packet_length()
{
    imu.packet_length = 0;

    if (imu.gyro_enabled)
    {
        imu.packet_length += 3 * sizeof(float);
    }
    if (imu.accel_enabled)
    {
        imu.packet_length += 3 * sizeof(float);
    }
    if (imu.mag_enabled)
    {
        imu.packet_length += 3 * sizeof(float);
    }
    if (imu.quat6_enabled)
    {
        imu.packet_length += 4 * sizeof(float);
    }
    if (imu.quat9_enabled)
    {
        imu.packet_length += 4 * sizeof(float);
    }
    if (imu.euler_enabled)
    {
        imu.packet_length += 3 * sizeof(float);
    }

    NRF_LOG_INFO("Packet Len set to: %d", imu.packet_length);
}





void thingy_tes_c_evt_handler(ble_thingy_tes_c_t * p_ble_tes_c, ble_tes_c_evt_t * p_evt)
{

    switch (p_evt->evt_type)
    {
        ret_code_t err_code;

        case BLE_THINGY_TES_C_EVT_DISCOVERY_COMPLETE:
        {
            err_code = ble_tes_c_handles_assign(&m_thingy_tes_c[p_evt->conn_handle],
                                                p_evt->conn_handle,
                                                &p_evt->params.peer_db);
            NRF_LOG_INFO("Thingy Environment service discovered on conn_handle 0x%x.", p_evt->conn_handle);
            

            err_code = ble_tes_c_quaternion_notif_enable(&m_thingy_tes_c[p_evt->conn_handle]);
            APP_ERROR_CHECK(err_code);

        }
        break;

        case BLE_TMS_EVT_QUAT:
        {

            float quat_buff[4];
            uint32_t quat_buff_len = sizeof(quat_buff);

            #define FIXED_POINT_FRACTIONAL_BITS_QUAT        30

            quat_buff[0] = ((float)p_evt->params.value.quat_data.w / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));
            quat_buff[1] = ((float)p_evt->params.value.quat_data.x / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));
            quat_buff[2] = ((float)p_evt->params.value.quat_data.y / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));
            quat_buff[3] = ((float)p_evt->params.value.quat_data.z / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));
            
            
            NRF_LOG_INFO("quat: %d %d  %d  %d", (int)(quat_buff[0]*1000), (int)(quat_buff[1]*1000), (int)(quat_buff[2]*1000), (int)(quat_buff[3]*1000));

            // // Put the received data in FIFO buffer
            // err_code = app_fifo_write(&buffer.received_data_fifo, quat_buff, &quat_buff);
            // if (err_code == NRF_ERROR_NO_MEM)
            // {
            //     NRF_LOG_INFO("RECEIVED DATA FIFO BUFFER FULL!");
            // }
            // if (err_code == NRF_SUCCESS)
            // {
            //     // Signal to event handler to execute sprintf + start UART transmission
            //     // If there are already events in the queue
            //     if (imu.evt_scheduled > 0)
            //     {
            //         imu.evt_scheduled++;
            //     }
            //     // If there are not yet any events in the queue, schedule event. In gpiote_evt_sceduled all callbacks are called
            //     else
            //     {
            //         imu.evt_scheduled++;
            //         err_code = app_sched_event_put(0, 0, data_evt_sceduled);
            //         APP_ERROR_CHECK(err_code);
            //     }
            // }
        }

        default:
        {
            // NRF_LOG_DEBUG("thingy_tes_c_evt_handler DEFAULT");
        }
        break;
    }
}

static void thingy_tes_c_init(void)
{
    ret_code_t       err_code;

    ble_thingy_tes_c_init_t thingy_tes_c_init_obj;
    thingy_tes_c_init_obj.evt_handler =  thingy_tes_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_thingy_tes_c_init(&m_thingy_tes_c[i], &thingy_tes_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    //    uart_init();

    uart_dma_init();

    // Create a buffer for the FIFO
    uint16_t uart_dma_buffer_size = 2048;
    uint8_t uart_dma_buffer[uart_dma_buffer_size];

    // Create a buffer for the FIFO
    uint16_t received_data_buffer_size = 4096;
    uint8_t received_data_buffer[received_data_buffer_size];

    uint32_t err_code;
    // Initialize FIFO structure for use in UART DMA
    err_code = app_fifo_init(&buffer.uart_dma_difo, uart_dma_buffer, (uint16_t)sizeof(uart_dma_buffer));
    APP_ERROR_CHECK(err_code);

    // Initialize FIFO structure for collecting received data
    err_code = app_fifo_init(&buffer.received_data_fifo, received_data_buffer, (uint16_t)sizeof(received_data_buffer));
    APP_ERROR_CHECK(err_code);

    // Application scheduler (soft interrupt like)
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    ////////////////////////////////////
    //	uint32_t err_code;
    //	while(1){
    //			if (!nrf_drv_uart_tx_in_progress(&imu.uart))
    //			{
    //        do
    //        {
    //						char test[] = ("teststringhahahaaahhaha\n");
    //						err_code = nrf_drv_uart_tx(&imu.uart, (uint8_t *) test, sizeof(test)-1);
    //
    //            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
    //            {
    //								NRF_LOG_ERROR("nrf_drv_uart_tx failed");
    //                APP_ERROR_CHECK(err_code);
    //            }
    //        } while (err_code == NRF_ERROR_BUSY);
    //			}
    //		}
    ///////////////////////////////////

    buttons_leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
    conn_evt_len_ext_set(); // added for faster speed

    gatt_init();
    nus_c_init();


    thingy_tes_c_init();

    /* CHANGES ADDED */
    ble_conn_state_init();
    /* END ADDED CHANGES */

    scan_init();

    // BLUE LED as output
    //nrf_gpio_cfg_output(7);

    // Start execution.
    printf("BLE UART central example started.\r\n");
    NRF_LOG_INFO("BLE UART central example started.");
    scan_start();

    // TimeSync
    // Start TimeSync AFTER scan_start()
    // This is a temporary fix for a known bug where connection is constantly closed with error code 0x3E
    sync_timer_init();

    /////////////// LED BUTTON BLINK /////////////////////
    //		gpio_init();
    /////////////// LED BUTTON BLINK /////////////////////

    /* CHANGES */
    // TIMER DIY INIT
    //timer_diy_init();
    /* END CHANGES */

    // Check time needed to process data
    nrf_gpio_cfg_output(18);
    // Check active time of CPU
    nrf_gpio_cfg_output(19);
    // Check time of NRF_LOG_FLUSH
    nrf_gpio_cfg_output(20);
    // Check UART transmission
    nrf_gpio_cfg_output(22);
    // Sprintf timing
    nrf_gpio_cfg_output(10);


    NRF_LOG_DEBUG("DEBUG ACTIVE");



    // Enter main loop.
    for (;;)
    {
        // App scheduler: handle event in buffer
        app_sched_execute();

        nrf_gpio_pin_set(20);
        NRF_LOG_FLUSH();
        nrf_gpio_pin_clear(20);
        nrf_gpio_pin_clear(19);
        // idle_state_handle();
        nrf_gpio_pin_set(19);

        // nrf_delay_ms(1000);
        // NRF_LOG_INFO("Loop");

        // Toggle pin to check CPU activity
        nrf_gpio_pin_toggle(17);
    }
}

/*
NRF_SDH_BLE_GAP_DATA_LENGTH 251

NRF_SDH_BLE_GAP_EVENT_LENGTH 300

NRF_SDH_BLE_GATT_MAX_MTU_SIZE 247
*/
