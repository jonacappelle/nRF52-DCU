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




void config_send(IMU *imu)
{
    ret_code_t err_code;

    ble_tes_config_t config;
    config.gyro_enabled = imu->gyro_enabled;
    config.accel_enabled = imu->accel_enabled;
    config.mag_enabled = imu->mag_enabled;
    config.euler_enabled = imu->euler_enabled;
    config.quat6_enabled = imu->quat6_enabled;
    config.quat9_enabled = imu->quat9_enabled;
    config.motion_freq_hz = imu->frequency;
    config.wom_enabled = imu->wom;
    config.sync_enabled = imu->sync_enabled;
    config.stop = imu->stop;
    config.adc_enabled = imu->adc;

    // Get timestamp from master
    imu->sync_start_time = ts_timestamp_get_ticks_u64();

    // Send start signal to be 2 seconds later
    imu->sync_start_time = ( TIME_SYNC_TIMESTAMP_TO_USEC(imu->sync_start_time) / 1000 ) + 2000;

    imu->sync_start_time = ( TIME_SYNC_MSEC_TO_TICK(imu->sync_start_time) / 100 ) * 100;


    config.sync_start_time = imu->sync_start_time;
    NRF_LOG_INFO("sync_start_time: %d", imu->sync_start_time);
    

    NRF_LOG_INFO("config.adc_enabled %d", config.adc_enabled);

    // Send config to peripheral
    usr_ble_config_send(config);
}

void config_reset(IMU *imu)
{
    imu->gyro_enabled = 0;
    imu->accel_enabled = 0;
    imu->mag_enabled = 0;
    imu->quat6_enabled = 0;
    imu->quat9_enabled = 0;
    imu->euler_enabled = 0;
    imu->frequency = 0;
    imu->sync_enabled = 0;
    imu->stop = 0;
    imu->adc = 0;
}



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
            // uart_print("\n");
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
            break;

        case CMD_LIST:
            NRF_LOG_INFO("CMD_LIST received");

            uart_print("------------------------------------------\n");
            uart_print("Connected devices list:\n");

            // Get connection handles
            ble_conn_state_conn_handle_list_t conn_central_handles = ble_conn_state_central_handles();
            //You can iterate through the list of connection handles:

            NRF_LOG_INFO("conn_central_handles.len %d", conn_central_handles.len);
            for (uint32_t i = 0; i < conn_central_handles.len; i++)
            {
                uint16_t conn_handle = conn_central_handles.conn_handles[i];

                // Print Connected Devices
                uint8_t str[100];
                sprintf(str, "Sensor    %d  --> conn handle  %d\n", (i + 1), conn_handle);
                uart_print(str);
                // NRF_LOG_INFO("Connection handle: %d\n", (i+1), conn_handle);
                // nrf_delay_ms(1);
            }

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

                imu.sync_enabled = 1;

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

                imu.sync_enabled = 0;

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

            imu.adc = 1;

            break;

        case CMD_GYRO:
            NRF_LOG_INFO("CMD_GYRO received");
            // NRF_LOG_FLUSH();
            imu.gyro_enabled = 1;
            break;

        case CMD_ACCEL:
            NRF_LOG_INFO("CMD_ACCEL received");
            // NRF_LOG_FLUSH();
            imu.accel_enabled = 1;
            break;

        case CMD_MAG:
            NRF_LOG_INFO("CMD_MAG received");
            // NRF_LOG_FLUSH();
            imu.mag_enabled = 1;
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
                imu.quat6_enabled = 1;
                break;

            case CMD_QUAT9:
                NRF_LOG_INFO("CMD_QUAT9 received");
                // NRF_LOG_FLUSH();
                imu.quat9_enabled = 1;
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
            imu.euler_enabled = 1;
            break;

        case CMD_RESET:
            NRF_LOG_INFO("CMD_RESET received");

            uart_print("------------------------------------------\n");
            uart_print("Config reset.\n");
            uart_print("------------------------------------------\n");

            config_reset(&imu);
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
                    imu.frequency = 10;
                    // NRF_LOG_FLUSH();
                    break;

                case CMD_FREQ_50:
                    NRF_LOG_INFO("CMD_FREQ_50 received");
                    imu.frequency = 50;
                    // NRF_LOG_FLUSH();
                    break;

                case CMD_FREQ_100:
                    NRF_LOG_INFO("CMD_FREQ_100 received");
                    imu.frequency = 100;
                    // NRF_LOG_FLUSH();
                    break;

                case CMD_FREQ_200:
                    NRF_LOG_INFO("CMD_FREQ_200 received");
                    imu.frequency = 200;
                    // NRF_LOG_FLUSH();
                    break;

                case CMD_FREQ_225:
                    NRF_LOG_INFO("CMD_FREQ_225 received");
                    imu.frequency = 225;
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

            imu.wom = 1;
            break;

        case CMD_STOP:
            NRF_LOG_INFO("CMD_STOP received");
            uart_print("------------------------------------------\n");
            uart_print("Sampling stopped.\n");
            uart_print("------------------------------------------\n");

            ble_tes_config_t stop_config;
            memset(&stop_config, 0, sizeof(stop_config));

            imu.stop = 1;

            // Send configuration to all connected peripherals
            ble_send_config(&stop_config);

            break;

        case CMD_SEND:

            // Send config
            NRF_LOG_INFO("CMD_CONFIG_SEND received");

            config_send(&imu);

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

    // NRF_LOG_INFO("uart_rx_scheduled - stop");
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
        config_reset(&imu);
        config_send(&imu);

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



static void schedule(app_sched_event_handler_t handler)
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

static void print_packet_count(ble_tes_c_evt_t *p_evt)
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









static void buffers_init()
{
    ret_code_t err_code;

    // // Initialize FIFO structure for collecting received data
    err_code = app_fifo_init(&buffer.received_data_fifo, buffer.received_data_buffer, (uint16_t)sizeof(buffer.received_data_buffer));
    APP_ERROR_CHECK(err_code);
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
    buffers_init();

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
    services_init(thingy_tes_c_evt_handler);


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
