#include "usr_ble.h"

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
#include "ble_conn_state.h"


#include "usr_uart.h"


// #include <stdint.h>
// #include <string.h>
// #include "nrf.h"
// #include "ble_advdata.h"
// #include "ble_advertising.h"
// #include "ble_conn_params.h"
// #include "nrf_sdh.h"
// #include "nrf_sdh_soc.h"
// #include "nrf_sdh_ble.h"
// #include "nrf_ble_gatt.h"
// #include "nrf_ble_qwr.h"
// #include "app_timer.h"
// #include "ble_nus.h"
// #include "app_uart.h"
// #include "app_util_platform.h"
// #include "bsp_btn_ble.h"
// #include "nrf_pwr_mgmt.h"

#include "ble_tes_c.h"

#include "ble.h"

#include "app_scheduler.h"

// List of connected slaves
#include "sdk_mapped_flags.h"

#include "usr_util.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG 1  /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO 3 /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE 1024 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1024 /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA 0 //1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

/* CHANGES */
//BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
BLE_NUS_C_ARRAY_DEF(m_ble_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< BLE Nordic UART Service (NUS) client instances. */
/* END CHANGES */
BLE_TES_C_ARRAY_DEF(m_thingy_tes_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< Structure used to identify the battery service. */

NRF_BLE_GATT_DEF(m_gatt);        /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc); /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);        /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue, /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
//static uint16_t m_ble_nus_max_data_len = 247 - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */






void ble_send_config(ble_tes_config_t * stop_config)
{
    ret_code_t err_code;

    // Send config to peripheral
    for (uint8_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_tes_config_set(&m_thingy_tes_c[i], stop_config);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_INFO("ble_tes_config_set error %d", err_code);
        }
    }
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
void scan_start(void)
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
void scan_init(void)
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
        // ble_nus_data_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
        // NRF_LOG_INFO("Character received");
        // NRF_LOG_INFO("Receive counter:	%d", imu.received_packet_counter);
        // imu.received_packet_counter1++;
        break;

    case BLE_NUS_C_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected.");
        scan_start();
        break;
    }
}



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
    {
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
        memset(&m_db_disc,0,sizeof(m_db_disc)); // According to ble_db_discovery_start() documentation the database shall be zero initialized before use: CHANGED
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

        // Print to uart if device disconnects
        char str1[100];
        sprintf(str1, "Connected: %d\n", p_gap_evt->conn_handle);
        // uart_print("------------------------------------------\n");
        uart_print(str1);
        // uart_print("------------------------------------------\n");
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
    {
        // Print to uart if device disconnects
        char str2[100];
        sprintf(str2, "Disconnected: %d\n", p_gap_evt->conn_handle);
        // uart_print("------------------------------------------\n");
        uart_print(str2);
        // uart_print("------------------------------------------\n");

        NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                     p_gap_evt->conn_handle,
                     p_gap_evt->params.disconnected.reason);
    }
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

        // case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        //     // No system attributes have been stored.
        //     err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        //     APP_ERROR_CHECK(err_code);
        //     break;

    default:
        break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
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
    ret_code_t err_code;

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



/** @brief Function for initializing the database discovery module. */
void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}

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



void conn_evt_len_ext_set(void)
{
    ret_code_t err_code;
    ble_opt_t opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = 1;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}


void thingy_tes_c_init(ble_tes_c_evt_handler_t thingy_tes_c_evt_handler)
{
    ret_code_t err_code;

    ble_thingy_tes_c_init_t thingy_tes_c_init_obj;
    thingy_tes_c_init_obj.evt_handler = thingy_tes_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_thingy_tes_c_init(&m_thingy_tes_c[i], &thingy_tes_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
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

void services_init(ble_tes_c_evt_handler_t thingy_tes_c_evt_handler)
{
    // BLE NUS Service
    nus_c_init();

    // Motion Service
    thingy_tes_c_init(thingy_tes_c_evt_handler);

    // TODO: add Battery Service

}


void usr_ble_handles_assign(ble_thingy_tes_c_t *p_ble_tes_c, ble_tes_c_evt_t *p_evt)
{
    ret_code_t err_code;

    err_code = ble_tes_c_handles_assign(&m_thingy_tes_c[p_evt->conn_handle],
                                        p_evt->conn_handle,
                                        &p_evt->params.peer_db);
    NRF_LOG_INFO("Thingy Environment service discovered on conn_handle 0x%x.", p_evt->conn_handle);
}

void usr_enable_notif(ble_thingy_tes_c_t *p_ble_tes_c, ble_tes_c_evt_t *p_evt)
{
    ret_code_t err_code;

    // Enable notifications - in peripheral this equates to turning on the sensors
    err_code = ble_tes_c_quaternion_notif_enable(&m_thingy_tes_c[p_evt->conn_handle]);
    APP_ERROR_CHECK(err_code);
    err_code = ble_tes_c_adc_notif_enable(&m_thingy_tes_c[p_evt->conn_handle]);
    APP_ERROR_CHECK(err_code);
    err_code = ble_tes_c_euler_notif_enable(&m_thingy_tes_c[p_evt->conn_handle]);
    APP_ERROR_CHECK(err_code);
    err_code = ble_tes_c_raw_notif_enable(&m_thingy_tes_c[p_evt->conn_handle]);
    APP_ERROR_CHECK(err_code);
}

void usr_ble_config_send(ble_tes_config_t config)
{
    ret_code_t err_code;

    // Send config to peripheral
    for (uint8_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        // do
        // {
        err_code = ble_tes_config_set(&m_thingy_tes_c[i], &config);
        // if(err_code != NRF_SUCCESS)
        // {
        //     NRF_LOG_INFO("ble_tes_config_set error %d", err_code);
        // }
        NRF_LOG_INFO("ble_tes_config_set error %d", err_code);
        // }while(err_code == NRF_ERROR_RESOURCES);
    }
}

void usr_ble_disconnect()
{
    ret_code_t err_code;

    for (int c = 0; c < NRF_SDH_BLE_CENTRAL_LINK_COUNT; c++)
    {
        err_code = sd_ble_gap_disconnect(m_ble_nus_c[c].conn_handle,
                                            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }    
}

