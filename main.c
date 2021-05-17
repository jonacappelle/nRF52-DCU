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
 *         File: main.c
 *      Created: 14-5-2021
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: Main file for NOMADe receiver (DCU)
 *
 *  Commissiond by NOMADe
 * 
 */

#include "main.h"

#include "usr_ble.h"


void ts_imu_trigger_enable(void);
void ts_imu_trigger_disable(void);



void uart_rx_scheduled(void *p_event_data, uint16_t event_size)
{
    // NRF_LOG_INFO("uart_rx_scheduled - start");

    // uint8_t state = CMD_TYPE;

    ret_code_t err_code;

    uint8_t p_byte[3];

    // If there is data left in FIFO and the read byte is not a carriage return
    while ((uart_rx_buff_get(p_byte) != NRF_ERROR_NOT_FOUND))
    {
        // NRF_LOG_INFO("FIFO get");

        // Check if end of message is reached
        if (p_byte[0] == CMD_CR)
        {
            // NRF_LOG_INFO("Break!");
            // NRF_LOG_FLUSH();
            break;
        }
        // Here we can process the request received over UART

        // NRF_LOG_INFO("p_byte: %d - %s", p_byte[0], p_byte[0]);
        // NRF_LOG_FLUSH();

        switch (p_byte[0])
        {
        case CMD_PRINT:
            NRF_LOG_INFO("CMD_PRINT received");

            uart_print("------------------------------------------\n");
            uart_print("-----  NOMADE WIRELESS SENSOR NODE   -----\n");
            uart_print("------------------------------------------\n");
            uart_print("\n");
            uart_print("Press:  'h' for help\n");
            uart_print("Press:  's' to show current settings\n");
            uart_print("Press:  '1' to set up sync\n");
            uart_print("Press:  'g' to enable gyroscope\n");
            uart_print("Press:  'a' to enable accelerometer\n");
            uart_print("Press:  'm' to enable magnetometer\n");
            uart_print("Press:  'e' to enable euler angles\n");
            uart_print("Press:  'q6' to enable 6 DoF quaternions\n");
            uart_print("Press:  'q9' to enable 9 DoF quaternions\n");
            uart_print("Press:  't' to stop sampling\n");
            uart_print("------------------------------------------\n");
            uart_print("Press:  'f' + '3 digital number' to set sampling frequency\n");
            uart_print("------------------------------------------\n");
            uart_print("Example:    q6f225  Enable 6 DoF Quaternions with sampling rate of 225 Hz\n");
            uart_print("------------------------------------------\n");
            break;

        case CMD_SETTINGS:
            NRF_LOG_INFO("CMD_SETTINGS received");
            usr_ble_print_settings();
            break;

        case CMD_LIST:
            NRF_LOG_INFO("CMD_LIST received");

            uart_print("------------------------------------------\n");
            uart_print("Connected devices list:\n");

            usr_ble_print_connection_handles();

            // uart_print("This feature is in progress...\n");
            uart_print("------------------------------------------\n");

            break;

        case CMD_SYNC:
            NRF_LOG_INFO("CMD_SYNC received");

            // Get byte after sync command
            uint8_t byte2[1];
            err_code = uart_rx_buff_get(byte2);

            switch (byte2[0])
            {
            case CMD_SYNC_ENABLE:
                NRF_LOG_INFO("CMD_SYNC_ENABLE received");

                // Start synchronization
                err_code = ts_tx_start(TIME_SYNC_FREQ_AUTO); //TIME_SYNC_FREQ_AUTO
                // err_code = ts_tx_start(2);
                APP_ERROR_CHECK(err_code);
                // ts_gpio_trigger_enable();
                ts_imu_trigger_enable();
                NRF_LOG_INFO("Starting sync beacon transmission!\r\n");

                set_config_sync_enable(1);

                uart_print("------------------------------------------\n");
                uart_print("Synchonization started.\n");
                uart_print("------------------------------------------\n");

                break;

            case CMD_SYNC_DISABLE:
                NRF_LOG_INFO("CMD_SYNC_DISABLE received");

                // Stop synchronization
                err_code = ts_tx_stop();
                ts_imu_trigger_disable();
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");

                set_config_sync_enable(0);

                uart_print("------------------------------------------\n");
                uart_print("Synchonization stopped.\n");
                uart_print("------------------------------------------\n");

                break;

            default:
                NRF_LOG_INFO("Invalid character after CMD_SYNC");
                break;
            }

            break;

        case CMD_ADC:
            NRF_LOG_INFO("CMD_ADC received");

            uart_print("------------------------------------------\n");
            uart_print("ADC enabled.\n");
            uart_print("------------------------------------------\n");

            set_config_adc_enable(1);

            break;

        case CMD_GYRO:
            NRF_LOG_INFO("CMD_GYRO received");
            // NRF_LOG_FLUSH();
            set_config_gyro_enable(1);
            break;

        case CMD_ACCEL:
            NRF_LOG_INFO("CMD_ACCEL received");
            // NRF_LOG_FLUSH();
            set_config_accel_enable(1);
            break;

        case CMD_MAG:
            NRF_LOG_INFO("CMD_MAG received");
            // NRF_LOG_FLUSH();
            set_config_mag_enable(1);
            break;

        case CMD_QUAT:
            NRF_LOG_INFO("CMD_QUAT received");
            // NRF_LOG_FLUSH();

            uint8_t byte[1];
            err_code = uart_rx_buff_get(byte);

            switch (byte[0])
            {
            case CMD_QUAT6:
                NRF_LOG_INFO("CMD_QUAT6 received");
                // NRF_LOG_FLUSH();
                set_config_quat6_enable(1);
                break;

            case CMD_QUAT9:
                NRF_LOG_INFO("CMD_QUAT9 received");
                // NRF_LOG_FLUSH();
                set_config_quat9_enable(1);
                break;

            default:
                NRF_LOG_INFO("Invalid character after CMD_QUAT");
                // NRF_LOG_FLUSH();
                break;
            }
            break;

        case CMD_EULER:
            NRF_LOG_INFO("CMD_EULER received");
            // NRF_LOG_FLUSH();
            set_config_euler_enable(1);
            break;

        case CMD_RESET:
            NRF_LOG_INFO("CMD_RESET received");

            uart_print("------------------------------------------\n");
            uart_print("Config reset.\n");
            uart_print("------------------------------------------\n");

            set_config_reset();
            break;

        case CMD_FREQ:
        {
            NRF_LOG_INFO("CMD_FREQ");
            // NRF_LOG_FLUSH();

            uint32_t cmd_freq_len = 3;

            uint8_t p_byte1[3];
            err_code = uart_rx_buff_read(p_byte1, &cmd_freq_len);

            // Get frequency components
            if (err_code == NRF_SUCCESS)
            {
                // NRF_LOG_INFO("success");
                // NRF_LOG_FLUSH();
                uint8_t cmd = uart_rx_to_cmd(p_byte1, CMD_FREQ_LEN);
                NRF_LOG_INFO("Frequency received: %d", cmd);

                switch (cmd)
                {
                case CMD_FREQ_10:
                    NRF_LOG_INFO("CMD_FREQ_10 received");
                    set_config_frequency(10);
                    // NRF_LOG_FLUSH();
                    break;

                case CMD_FREQ_50:
                    NRF_LOG_INFO("CMD_FREQ_50 received");
                    set_config_frequency(50);
                    // NRF_LOG_FLUSH();
                    break;

                case CMD_FREQ_100:
                    NRF_LOG_INFO("CMD_FREQ_100 received");
                    set_config_frequency(100);
                    // NRF_LOG_FLUSH();
                    break;

                case CMD_FREQ_200:
                    NRF_LOG_INFO("CMD_FREQ_200 received");
                    set_config_frequency(200);
                    // NRF_LOG_FLUSH();
                    break;

                case CMD_FREQ_225:
                    NRF_LOG_INFO("CMD_FREQ_225 received");
                    set_config_frequency(225);
                    // NRF_LOG_FLUSH();
                    break;

                default:
                    NRF_LOG_INFO("Invalid character CMD_FREQ");
                    uart_print("Invalid frequency selected!\n");
                    NRF_LOG_FLUSH();
                    break;
                }
            }
            else
            {
                NRF_LOG_INFO("err_code: %d", err_code);
                // NRF_LOG_FLUSH();
            }
        }
        break;

        case CMD_WOM:
            NRF_LOG_INFO("CMD_WOM received");
            uart_print("------------------------------------------\n");
            uart_print("Wake On Motion Enabled.\n");
            uart_print("------------------------------------------\n");

            set_config_wom_enable(1);
            break;

        case CMD_SEND:

            // Send config
            NRF_LOG_INFO("CMD_CONFIG_SEND received");

            config_send();

            uart_print("------------------------------------------\n");
            uart_print("Configuration send to peripherals.\n");
            uart_print("------------------------------------------\n");
            break;

        default:
            NRF_LOG_INFO("DEFAULT");
            NRF_LOG_FLUSH();
            uart_print("------------------------------------------\n");
            uart_print("Invalid command.\n");
            uart_print("------------------------------------------\n");
            break;
        }
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


static void ts_evt_callback(const ts_evt_t *evt)
{

    APP_ERROR_CHECK_BOOL(evt != NULL);

    switch (evt->type)
    {
    case TS_EVT_SYNCHRONIZED:
        NRF_LOG_INFO("TS_EVT_SYNCHRONIZED");
        // ts_gpio_trigger_enable();
        ts_imu_trigger_enable();
        break;
    case TS_EVT_DESYNCHRONIZED:
        NRF_LOG_INFO("TS_EVT_DESYNCHRONIZED");
        // ts_gpio_trigger_disable();
        ts_imu_trigger_disable();
        break;
    case TS_EVT_TRIGGERED:
        // NRF_LOG_INFO("TS_EVT_TRIGGERED");
        if (ts_get_imu_trigger_enabled())
        {
            uint32_t tick_target;

            tick_target = evt->params.triggered.tick_target + 4;

            // NRF_LOG_INFO("tick_target %d", tick_target);

            ret_code_t err_code = ts_set_trigger(tick_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));

            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("ts_evt_callback ERROR: %d", err_code);
                NRF_LOG_FLUSH();
            }
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            // Ensure pin is low when triggering is stopped
            nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);
        }
        uint64_t time_now_ticks;
        uint32_t time_now_msec;
        time_now_ticks = ts_timestamp_get_ticks_u64();
        time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;
        // NRF_LOG_INFO("Time: %d", time_now_msec);
        break;
    default:
        APP_ERROR_CHECK_BOOL(false);
        break;
    }
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
        //     static bool m_send_sync_pkt = false;

        //     if (m_send_sync_pkt)
        //     {
        //         m_send_sync_pkt = false;
        //         m_gpio_trigger_enabled = false;

        //         // bsp_board_leds_off();

        //         err_code = ts_tx_stop();
        //         APP_ERROR_CHECK(err_code);

        //         NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");
        //     }
        //     else
        //     {
        //         m_send_sync_pkt = true;

        //         // bsp_board_leds_on();

        //         APP_ERROR_CHECK(err_code);
        //         // err_code = ts_tx_start(TIME_SYNC_FREQ_AUTO);
        //         err_code = ts_tx_start(2);

        //         // ts_gpio_trigger_enable();
        //         ts_imu_trigger_enable();

        //         NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
        //     }

        // Clear config and send it: Stop measurements
        set_config_reset();
        config_send();

        NRF_LOG_INFO("BSP KEY 0: SENSORS STOP!");

    }
    break;
        // TimeSync end

    case BSP_EVENT_KEY_1:
    {
        // uint8_t temp_config1[] = {ENABLE_GYRO, ENABLE_ACCEL, ENABLE_QUAT6};
        // config_imu(temp_config1, sizeof(temp_config1));
        // break;

        break;
    }
    case BSP_EVENT_KEY_2:
    {
        // uint8_t temp_config3[] = {ENABLE_GYRO, ENABLE_ACCEL};
        // config_imu(temp_config3, sizeof(temp_config3));
        // break;
        break;
    }
    case BSP_EVENT_KEY_3:
    {
        uint64_t time_ticks;
        uint32_t time_usec;

        time_ticks = ts_timestamp_get_ticks_u64();
        time_usec = TIME_SYNC_TIMESTAMP_TO_USEC(time_ticks);

        NRF_LOG_INFO("Timestamp: %d us (%d, %d)", time_usec, time_usec / 1000000, time_usec / 1000);
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


        // Disconnect BLE
        usr_ble_disconnect();

        /* END CHANGES */

        break;

    default:
        break;
    }
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

    NRF_LOG_INFO("BLE DCU central started.");
    NRF_LOG_DEBUG("DEBUG ACTIVE");
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
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




int main(void)
{
    ret_code_t err_code;

    // Initialize.
    log_init();
    timer_init();
    //    uart_init();

    // A better UART driver than nrf_uart_drv - asynchronous with DMA and QUEUE
    libuarte_init(uart_rx_scheduled);

    // uart_dma_init();

    // Initialize buffers
    received_data_buffers_init();

    // Application scheduler (soft interrupt like)
    scheduler_init();

    buttons_leds_init();

    db_discovery_init();
    power_management_init();
    ble_stack_init();
    
    // Not needed - by setting correct settings in sdk_config.h this will be enabled
    // conn_evt_len_ext_set(); // added for faster speed

    gatt_init();

    // Initialize BLE services
    services_init();

    // Reset BLE connection state
    /* CHANGES ADDED */
    ble_conn_state_init();
    /* END ADDED CHANGES */

    // Init scanning for devices with NUS service + start scanning
    scan_init();
    scan_start();

    // TimeSync
    // Start TimeSync AFTER scan_start()
    // This is a temporary fix for a known bug where connection is constantly closed with error code 0x3E
    sync_timer_init(ts_evt_callback);

    // Initialize pins for debugging
    usr_gpio_init();

    // Enter main loop.
    for (;;)
    {
        // App scheduler: handle event in buffer
        app_sched_execute();

        // RTT Logging
        NRF_LOG_FLUSH();

        // Run power management
        idle_state_handle();

        // Toggle pin to check CPU activity
        // check_cpu_activity();
    }
}
