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

#include "time_sync.h"


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

// Initialisation of struct to keep track of different buffers
BUFFER buffer;


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



static void queue_process_packet(received_data_t * data, uint32_t * len)
{
    ret_code_t err_code;

    // Put the received data in FIFO buffer
    err_code = app_fifo_write(&buffer.received_data_fifo, (uint8_t *) data, len);

    NRF_LOG_INFO("Fifo PUT: %d", usr_get_fifo_len(&buffer.received_data_fifo))

    if (err_code == NRF_ERROR_NO_MEM)
    {
        NRF_LOG_INFO("RECEIVED DATA FIFO BUFFER FULL!");
    }
    if (err_code == NRF_SUCCESS)
    {
        schedule(imu_uart_sceduled); // TODO CHECK
    }
    APP_ERROR_CHECK(err_code);
}

void print_packet_count(ble_tes_c_evt_t *p_evt)
{
    // Print out the packet count from each of the slaves
    if (p_evt->conn_handle == 0)
    {
        imu.received_packet_counter1++;
        NRF_LOG_INFO("received_packet_counter1 %d", imu.received_packet_counter1);
    }
    else if (p_evt->conn_handle == 1)
    {
        imu.received_packet_counter2++;
        NRF_LOG_INFO("received_packet_counter2 %d", imu.received_packet_counter2);
    }
    else if (p_evt->conn_handle == 2)
    {
        imu.received_packet_counter3++;
        NRF_LOG_INFO("received_packet_counter3 %d", imu.received_packet_counter3);
    }
    else if (p_evt->conn_handle == 3)
    {
        imu.received_packet_counter4++;
        NRF_LOG_INFO("received_packet_counter4 %d", imu.received_packet_counter4);
    }
}



void thingy_tes_c_evt_handler(ble_thingy_tes_c_t *p_ble_tes_c, ble_tes_c_evt_t *p_evt)
{

    nrf_gpio_pin_set(11);

    // Print packet count for each connected device
    print_packet_count(p_evt);

    switch (p_evt->evt_type)
    {
        ret_code_t err_code;

    case BLE_THINGY_TES_C_EVT_DISCOVERY_COMPLETE:
    {
        // Assign connection handles
        usr_ble_handles_assign(p_ble_tes_c, p_evt);

        // Enable notifications - in peripheral this equates to turning on the sensors
        usr_enable_notif(p_ble_tes_c, p_evt);
    }
    break;

    case BLE_TMS_EVT_QUAT:
    {

        for (uint8_t i = 0; i < BLE_PACKET_BUFFER_COUNT; i++)
        {

            received_data_t received_quat;
            uint32_t received_quat_len = sizeof(received_quat);

            // Initialize struct to all zeros
            memset(&received_quat, 0, received_quat_len);

    #define FIXED_POINT_FRACTIONAL_BITS_QUAT 30

            received_quat.conn_handle = p_evt->conn_handle;
            received_quat.quat_data_present = 1;
            received_quat.quat_data.w = ((float)p_evt->params.value.quat_data.quat[i].w / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));
            received_quat.quat_data.x = ((float)p_evt->params.value.quat_data.quat[i].x / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));
            received_quat.quat_data.y = ((float)p_evt->params.value.quat_data.quat[i].y / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));
            received_quat.quat_data.z = ((float)p_evt->params.value.quat_data.quat[i].z / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));

            // NRF_LOG_INFO("quat: %d %d  %d  %d", (int)(quat_buff[0]*1000), (int)(quat_buff[1]*1000), (int)(quat_buff[2]*1000), (int)(quat_buff[3]*1000));

            // Put data into FIFO buffer and let event handler know to process the packet
            queue_process_packet(&received_quat, &received_quat_len);
        }
    }
    break;

    case BLE_TMS_EVT_EULER:
    {
        float euler_buff[3];
        uint32_t euler_buff_len = sizeof(euler_buff);

#define FIXED_POINT_FRACTIONAL_BITS_EULER 16

        euler_buff[0] = ((float)p_evt->params.value.euler_data.yaw / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_EULER));
        euler_buff[1] = ((float)p_evt->params.value.euler_data.pitch / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_EULER));
        euler_buff[2] = ((float)p_evt->params.value.euler_data.roll / (float)(1 << FIXED_POINT_FRACTIONAL_BITS_EULER));

        NRF_LOG_INFO("euler: %d %d  %d", (int)euler_buff[0], (int)euler_buff[1], (int)euler_buff[2]);
    }
    break;

    case BLE_TMS_EVT_RAW:
    {

        for (uint8_t i = 0; i < BLE_PACKET_BUFFER_COUNT; i++)
        {

    #define RAW_Q_FORMAT_GYR_COMMA_BITS 5  // Number of bits used for comma part of raw data.
    #define RAW_Q_FORMAT_ACC_COMMA_BITS 10 // Number of bits used for comma part of raw data.
    #define RAW_Q_FORMAT_CMP_COMMA_BITS 4  // Number of bits used for comma part of raw data.

            received_data_t received_raw;
            uint32_t received_raw_len = sizeof(received_raw);

            // Initialize struct to all zeros
            memset(&received_raw, 0, received_raw_len);

            received_raw.conn_handle = p_evt->conn_handle;
            received_raw.raw_data_present = 1;
            received_raw.raw_data.gryo.x = ((float)p_evt->params.value.raw_data.single_raw[i].gyro.x / (float)(1 << RAW_Q_FORMAT_GYR_COMMA_BITS));
            received_raw.raw_data.gryo.y = ((float)p_evt->params.value.raw_data.single_raw[i].gyro.y / (float)(1 << RAW_Q_FORMAT_GYR_COMMA_BITS));
            received_raw.raw_data.gryo.z = ((float)p_evt->params.value.raw_data.single_raw[i].gyro.z / (float)(1 << RAW_Q_FORMAT_GYR_COMMA_BITS));

            received_raw.raw_data.accel.x = ((float)p_evt->params.value.raw_data.single_raw[i].accel.x / (float)(1 << RAW_Q_FORMAT_ACC_COMMA_BITS));
            received_raw.raw_data.accel.y = ((float)p_evt->params.value.raw_data.single_raw[i].accel.y / (float)(1 << RAW_Q_FORMAT_ACC_COMMA_BITS));
            received_raw.raw_data.accel.z = ((float)p_evt->params.value.raw_data.single_raw[i].accel.z / (float)(1 << RAW_Q_FORMAT_ACC_COMMA_BITS));

            received_raw.raw_data.mag.x = ((float)p_evt->params.value.raw_data.single_raw[i].compass.x / (float)(1 << RAW_Q_FORMAT_CMP_COMMA_BITS));
            received_raw.raw_data.mag.y = ((float)p_evt->params.value.raw_data.single_raw[i].compass.y / (float)(1 << RAW_Q_FORMAT_CMP_COMMA_BITS));
            received_raw.raw_data.mag.z = ((float)p_evt->params.value.raw_data.single_raw[i].compass.z / (float)(1 << RAW_Q_FORMAT_CMP_COMMA_BITS));

            // Put data into FIFO buffer and let event handler know to process the packet
            queue_process_packet(&received_raw, &received_raw_len);

            // NRF_LOG_INFO("raw:  gyro: %d %d  %d", (int)(received_raw.raw_data.gryo.x*1000), (int)(received_raw.raw_data.gryo.y*1000), (int)(received_raw.raw_data.gryo.z*1000));
            // NRF_LOG_INFO("raw:  accel: %d   %d  %d", (int)(accel[0]*1000), (int)(accel[1]*1000), (int)(accel[2]*1000));
            // NRF_LOG_INFO("raw:  mag: %d %d  %d", (int)(mag[0]*1000), (int)(mag[1]*1000), (int)(mag[2]*1000));

            // NRF_LOG_INFO("raw:  gyro: %d %d  %d", (int)(gyro[0]*1000), (int)(gyro[1]*1000), (int)(gyro[2]*1000));
            // NRF_LOG_INFO("raw:  accel: %d   %d  %d", (int)(accel[0]*1000), (int)(accel[1]*1000), (int)(accel[2]*1000));
            // NRF_LOG_INFO("raw:  mag: %d %d  %d", (int)(mag[0]*1000), (int)(mag[1]*1000), (int)(mag[2]*1000));
        }
    }
    break;

    case BLE_TMS_EVT_ADC:
    {
        // NRF_LOG_INFO("ADC data: %d", p_evt->params.value.adc_data.raw[1]);

        // received_data_t received_adc;
        // uint32_t received_adc_len = sizeof(received_adc);

        // // Initialize struct to all zeros
        // memset(&received_adc, 0, received_adc_len);

        // received_adc.conn_handle = p_evt->conn_handle;
        // received_adc.adc_data_present = 1;

        // // TODO copy all data to print buffers
        // received_adc.adc_data.raw[0] = p_evt->params.value.adc_data.raw[0];

        // // Put data into FIFO buffer and let event handler know to process the packet
        // queue_process_packet(&received_adc, &received_adc_len);
    }
    break;

    default:
    {
        NRF_LOG_INFO("thingy_tes_c_evt_handler DEFAULT: %d", (p_evt->evt_type));
    }
    break;
    }

    nrf_gpio_pin_clear(11);
}

void thingy_tes_c_init()
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

void services_init()
{
    // BLE NUS Service
    nus_c_init();

    // Motion Service
    thingy_tes_c_init();

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





void received_data_buffers_init()
{
    ret_code_t err_code;

    // // Initialize FIFO structure for collecting received data
    err_code = app_fifo_init(&buffer.received_data_fifo, buffer.received_data_buffer, (uint16_t)sizeof(buffer.received_data_buffer));
    APP_ERROR_CHECK(err_code);
}


void schedule(app_sched_event_handler_t handler)
{
    ret_code_t err_code;

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
        err_code = app_sched_event_put(0, 0, imu_uart_sceduled);
        APP_ERROR_CHECK(err_code);
    }
}


void imu_uart_sceduled(void *p_event_data, uint16_t event_size)
{
    while (imu.evt_scheduled > 0)
    {
        ret_code_t err_code;

        bool read_success = false;

        // NRF_LOG_INFO("App scheduler execute: %d", imu.evt_scheduled);

        char string[1024];

        received_data_t temp;
        uint32_t temp_len = sizeof(temp);

        float quat[4];
        uint32_t quat_len = sizeof(quat);

        if (app_fifo_read(&buffer.received_data_fifo, (uint8_t *)&temp, &temp_len) == NRF_SUCCESS)
        {

            NRF_LOG_INFO("Fifo GET: %d", usr_get_fifo_len(&buffer.received_data_fifo))

            // sprintf(string, "%d w%.3fwa%.3fab%.3fbc%.3fc\n", device_nr[0], quat[0], quat[1], quat[2], quat[3]);

            // If packet contains QUATERNIONS
            if (temp.quat_data_present)
            {
                sprintf(string, "%d Q   %.3f    %.3f    %.3f    %.3f\n", temp.conn_handle, temp.quat_data.w, temp.quat_data.x, temp.quat_data.y, temp.quat_data.z);
            }
            else
                // if packet contains RAW DATA
                if (temp.raw_data_present)
            {
                sprintf(string, "%d G %.3f %.3f %.3f        A %.3f %.3f %.3f        M %.3f %.3f %.3f\n", temp.conn_handle, temp.raw_data.gryo.x, temp.raw_data.gryo.y, temp.raw_data.gryo.z, temp.raw_data.accel.x, temp.raw_data.accel.y, temp.raw_data.accel.z, temp.raw_data.mag.x, temp.raw_data.mag.y, temp.raw_data.mag.z);
            }
            // else
                // If packet contains ADC DATA
            //     if (temp.adc_data_present)
            // {
            //     sprintf(string, "%d ADC %.3f\n", temp.conn_handle, temp.adc_data.raw[0]);
            //     NRF_LOG_INFO("To be implemented.");
            // }

            read_success = true;
            imu.evt_scheduled--;
        }

        // Get data from FIFO buffer if data is correctly recognized
        if (read_success)
        {
            uint32_t string_len = calculate_string_len(string);

            // Send data over UART
            uart_queued_tx((uint8_t *)string, &string_len);
        }
    }
}


void usr_ble_print_settings()
{
    uart_print("------------------------------------------\n");
    uart_print("Current settings:\n");
    if (imu.gyro_enabled)
        uart_print("---    Gyroscope enabled\n");
    if (imu.accel_enabled)
        uart_print("---   Accelerometer enabled\n");
    if (imu.mag_enabled)
        uart_print("--- Magnetometer enabled\n");
    if (imu.euler_enabled)
        uart_print("---   Euler angles enabled\n");
    if (imu.quat6_enabled)
        uart_print("---   Quaternions 6 DoF enabled\n");
    if (imu.quat9_enabled)
        uart_print("---   Quaternions 9 DoF enabled\n");
    if (imu.frequency != 0)
    {
        uart_print("---  Sensor frequency:  ");
        char str[5];
        sprintf(str, "%d Hz\n", imu.frequency);
        NRF_LOG_INFO("string: %s", str);
        uart_print(str);
    }
    if (imu.adc)
        uart_print("---   ADC enabled\n");
    if (imu.sync_enabled)
        uart_print("---   Synchonization enabled\n");
    uart_print("------------------------------------------\n");
}


void usr_ble_print_connection_handles()
{
    ble_conn_state_conn_handle_list_t conn_central_handles = ble_conn_state_central_handles();

    //You can iterate through the list of connection handles:
    for (uint32_t i = 0; i < conn_central_handles.len; i++)
    {
        uint16_t conn_handle = conn_central_handles.conn_handles[i];

        // Print Connected Devices
        uint8_t str[100];
        sprintf(str, "Sensor    %d  --> conn handle  %d\n", (i + 1), conn_handle);
        uart_print(str);
    }
}


void set_config_sync_enable(bool enable)
{
    imu.sync_enabled = enable;
}

void set_config_adc_enable(bool enable)
{
    imu.adc = enable;
}

void set_config_gyro_enable(bool enable)
{
    imu.gyro_enabled = enable;
}

void set_config_accel_enable(bool enable)
{
    imu.accel_enabled = enable;
}

void set_config_mag_enable(bool enable)
{
    imu.mag_enabled = enable;
}

void set_config_quat6_enable(bool enable)
{
    imu.quat6_enabled = enable;
}

void set_config_quat9_enable(bool enable)
{
    imu.quat9_enabled = enable;
}

void set_config_euler_enable(bool enable)
{
    imu.euler_enabled = enable;
}

void set_config_wom_enable(bool enable)
{
    imu.wom = enable;
}

void set_config_frequency(uint32_t freq)
{
    imu.frequency = freq;
}

void set_config_reset()
{
    imu.gyro_enabled = 0;
    imu.accel_enabled = 0;
    imu.mag_enabled = 0;
    imu.quat6_enabled = 0;
    imu.quat9_enabled = 0;
    imu.euler_enabled = 0;
    imu.frequency = 0;
    imu.sync_enabled = 0;
    imu.stop = 0;
    imu.adc = 0;
}

void config_send()
{
    ret_code_t err_code;

    ble_tes_config_t config;
    config.gyro_enabled = imu.gyro_enabled;
    config.accel_enabled = imu.accel_enabled;
    config.mag_enabled = imu.mag_enabled;
    config.euler_enabled = imu.euler_enabled;
    config.quat6_enabled = imu.quat6_enabled;
    config.quat9_enabled = imu.quat9_enabled;
    config.motion_freq_hz = imu.frequency;
    config.wom_enabled = imu.wom;
    config.sync_enabled = imu.sync_enabled;
    config.stop = imu.stop;
    config.adc_enabled = imu.adc;

    // Get timestamp from master
    imu.sync_start_time = ts_timestamp_get_ticks_u64();

    // Send start signal to be 2 seconds later
    imu.sync_start_time = ( TIME_SYNC_TIMESTAMP_TO_USEC(imu.sync_start_time) / 1000 ) + 2000;

    imu.sync_start_time = ( TIME_SYNC_MSEC_TO_TICK(imu.sync_start_time) / 100 ) * 100;


    config.sync_start_time = imu.sync_start_time;
    NRF_LOG_INFO("sync_start_time: %d", imu.sync_start_time);
    

    NRF_LOG_INFO("config.adc_enabled %d", config.adc_enabled);

    // Send config to peripheral
    usr_ble_config_send(config);
}

