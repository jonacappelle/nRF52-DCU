#include "usr_util.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_delay.h"

#include "app_fifo.h"

#include "app_scheduler.h"

#include "usr_uart.h"
#include "usr_ble.h"
#include "time_sync.h"
#include "usr_time_sync.h"

#include "nrf_pwr_mgmt.h"

#include "bsp_btn_ble.h"

extern BATTERY_ARRAY batt_array;

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

    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
        NRF_LOG_INFO("Power management wants to reset to DFU mode.");
        // APP_ERROR_HANDLER(NRF_ERROR_API_NOT_IMPLEMENTED);
        break;

    default:
        break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


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
        // Clear config and send it: Stop measurements
        set_config_reset();
        config_send();

        NRF_LOG_INFO("BSP KEY 0: SENSORS STOP!");
    }
    break;
        // TimeSync end

    case BSP_EVENT_KEY_1:
    {
        break;
    }
    case BSP_EVENT_KEY_2:
    {
        break;
    }
    case BSP_EVENT_KEY_3:
    {
        // Print synchronized time
        ts_print_sync_time();
        break;
    }
    case BSP_EVENT_SLEEP:
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        break;

    case BSP_EVENT_DISCONNECT:
    {
        // Disconnect BLE
        usr_ble_disconnect();
        break;
    }
    default:
        break;
    }
}


/**@brief Function for initializing buttons and leds. */
void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    // err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);

    // TimeSync begin
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    // err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    // TimeSync end

    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}

void usr_gpio_init()
{
    // Check time needed to process data
    nrf_gpio_cfg_output(18);
    // Check active time of CPU
    nrf_gpio_cfg_output(19);
    // Check time of NRF_LOG_FLUSH
    // nrf_gpio_cfg_output(20); // short with reset -> need to be fixed on pcb!
    // Check UART transmission
    nrf_gpio_cfg_output(22);
    // Sprintf timing
    nrf_gpio_cfg_output(10);

    nrf_gpio_cfg_output(11);
    nrf_gpio_cfg_output(12);

    nrf_gpio_cfg_output(PIN_CPU_ACTIVITY);


}

void check_cpu_activity()
{
    nrf_gpio_pin_toggle(PIN_CPU_ACTIVITY);
}



uint32_t calculate_string_len(char * string)
{
    uint32_t len = 0;
    do
    {
        len++;
    } while (string[len] != '\n');
    len++;

    return len;
}


// App scheduler
void scheduler_init()
{
    // Application scheduler (soft interrupt like)
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

// App FIFO
uint32_t usr_get_fifo_len(app_fifo_t * p_fifo)
{
    uint32_t tmp = p_fifo->read_pos;
    return p_fifo->write_pos - tmp;
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for initializing power management.
 */
void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module. */
void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("BLE DCU central started.");
    NRF_LOG_DEBUG("DEBUG ACTIVE");
}


// Configurator

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

        // case CMD_EULER:
        //     NRF_LOG_INFO("CMD_EULER received");
        //     // NRF_LOG_FLUSH();
        //     set_config_euler_enable(1);
        //     break;

        case CMD_BATT:
            NRF_LOG_INFO("CMD_BATT received");

            // for(uint8_t i=0; i<)

            // // if(batt.level != BATT_INVALID_VALUE)
            // // {
            //     uint8_t temp[10];
            //     sprintf(temp, "Battery level:   %0.2f   ( +- %d procent )\n", batt.voltage, batt.level);

                
            //     uart_print(temp);
                
            usr_batt_print_conn_handle();


            // }else{
            //     uart_print("------------------------------------------\n");
            //     uart_print("Battery level:   unknown\n");
            //     uart_print("------------------------------------------\n");  
            // }
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
                uint32_t cmd = uart_rx_to_cmd(p_byte1, CMD_FREQ_LEN);
                NRF_LOG_INFO("Frequency received: %d", cmd);

                set_config_frequency(cmd);
                NRF_LOG_INFO("CMD_FREQ set");

                // switch (cmd)
                // {
                // case CMD_FREQ_10:
                //     NRF_LOG_INFO("CMD_FREQ_10 received");
                //     set_config_frequency(10);
                //     // NRF_LOG_FLUSH();
                //     break;

                // case CMD_FREQ_50:
                //     NRF_LOG_INFO("CMD_FREQ_50 received");
                //     set_config_frequency(50);
                //     // NRF_LOG_FLUSH();
                //     break;

                // case CMD_FREQ_100:
                //     NRF_LOG_INFO("CMD_FREQ_100 received");
                //     set_config_frequency(100);
                //     // NRF_LOG_FLUSH();
                //     break;

                // case CMD_FREQ_200:
                //     NRF_LOG_INFO("CMD_FREQ_200 received");
                //     set_config_frequency(200);
                //     // NRF_LOG_FLUSH();
                //     break;

                // case CMD_FREQ_225:
                //     NRF_LOG_INFO("CMD_FREQ_225 received");
                //     set_config_frequency(225);
                //     // NRF_LOG_FLUSH();
                //     break;

                // default:
                //     NRF_LOG_INFO("Invalid character CMD_FREQ");
                //     uart_print("Invalid frequency selected!\n");
                //     NRF_LOG_FLUSH();
                //     break;
                // }
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
            
            uint32_t cmd_wom_len = 1;

            uint8_t p_byte_w[1];
            err_code = uart_rx_buff_read(p_byte_w, &cmd_wom_len);

            // Get frequency components
            if (err_code == NRF_SUCCESS)
            {
                uint8_t wom_state = uart_rx_to_cmd(p_byte_w, cmd_wom_len);

                // Set variabled to send accordingly
                set_config_wom_enable(wom_state);

                if( wom_state == 1 )
                {
                    uart_print("------------------------------------------\n");
                    uart_print("Wake On Motion Enabled.\n");
                    uart_print("------------------------------------------\n");
                }else if( wom_state == 0 ){
                    uart_print("------------------------------------------\n");
                    uart_print("Wake On Motion Disabled.\n");
                    uart_print("------------------------------------------\n");
                }else{
                    uart_print("------------------------------------------\n");
                    uart_print("Invalid WOM command.\n");
                    uart_print("------------------------------------------\n");
                }
            }
            break;

        case CMD_SEND:

            // Send config
            NRF_LOG_INFO("CMD_CONFIG_SEND received");

            config_send();

            uart_print("------------------------------------------\n");
            uart_print("Configuration send to peripherals.\n");
            uart_print("------------------------------------------\n");
            break;

        case CMD_DISCONNECT:
            NRF_LOG_INFO("CMD_DISCONNECT received");

            uint32_t cmd_disconnect_len = 1;

            uint8_t p_byte_d[3];
            err_code = uart_rx_buff_read(p_byte_d, &cmd_disconnect_len);

            // Get frequency components
            if (err_code == NRF_SUCCESS)
            {
                uint8_t conn_handle = uart_rx_to_cmd(p_byte_d, cmd_disconnect_len);

                err_code = imu_disconnect(conn_handle);
                if( err_code == BLE_ERROR_INVALID_CONN_HANDLE )
                {
                    uart_print("------------------------------------------\n");
                    uart_print("Invalid connection handle.\n");
                    uart_print("------------------------------------------\n");
                }else{
                    uart_print("------------------------------------------\n");
                    uart_print("Sensors disconnected.\n");
                    uart_print("------------------------------------------\n");
                }

                NRF_LOG_INFO("conn_handle %d disconnected", conn_handle);
            }

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


float usr_map_adc_to_uint8(uint8_t lvl)
{
    float voltage;
    
    // 1.4V range -> need to spread over uint8_t with max value 0xFF (255)
    float range = 4.2 - 2.8;

    voltage = (lvl / 255.0) * range;

    // Add 2.8V from total voltage
    // Measurement range of 2.8V - 4.2V
    voltage += 2.8;

    return voltage;
}


uint8_t usr_adc_voltage_to_percent(float voltage)
{
    if( voltage >= 4.1 ) return 100;
    else if ( voltage >= 4.0 ) return 90;
    else if ( voltage >= 3.9 ) return 70; 
    else if ( voltage >= 3.8 ) return 50;
    else if ( voltage >= 3.7 ) return 30;
    else if ( voltage >= 3.5 ) return 20;
    else if ( voltage >= 3.3 ) return 10;
    else return 0;
}



typedef enum
{
    USR_LED_0 = 11,
    USR_LED_1 = 12,
    USR_LED_2 = 13,
    USR_LED_3 = 14,
    USR_LED_4 = 15,
    USR_LED_5 = 17
} dcu_leds_t;

// Declare DCU LED pins
#define USR_LED_0 11
#define USR_LED_1 12
#define USR_LED_2 13
#define USR_LED_3 14
#define USR_LED_4 15
#define USR_LED_5 17

#define USR_NR_OF_LEDS 6

// Make list out of DCU LEDS
#define DCU_LEDS_LIST { USR_LED_0, USR_LED_1, USR_LED_2, USR_LED_3, USR_LED_4, USR_LED_5 }

#if USR_NR_OF_LEDS > 0
static const uint8_t dcu_led_list[USR_NR_OF_LEDS] = DCU_LEDS_LIST;
#endif

APP_TIMER_DEF(indication_led_timer);     /**< Handler for repeated timer used to blink LED 1. */

bool leds_on_off = 1;
uint32_t usr_leds_ctr = 0;

/**@brief Timeout handler for the repeated timer.
 */
static void indication_led_timer_handler(void * p_context)
{
    if(leds_on_off)
    {
        nrf_gpio_cfg_output(dcu_led_list[usr_leds_ctr]);
        nrf_gpio_pin_set(dcu_led_list[usr_leds_ctr]);
    }else if(leds_on_off == 0)
    {
        nrf_gpio_pin_clear(dcu_led_list[usr_leds_ctr]);
    }

    if(usr_leds_ctr == USR_NR_OF_LEDS)
    {
        usr_leds_ctr = 0;
        leds_on_off = !leds_on_off;
    }else{
       usr_leds_ctr++; 
    }

}



/**@brief Create timers.
 */
void create_timers()
{
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&indication_led_timer,
                                APP_TIMER_MODE_REPEATED,
                                indication_led_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(indication_led_timer, APP_TIMER_TICKS(100), NULL);
}

void dcu_leds_reset()
{

    for(uint8_t i = 0; i<=USR_NR_OF_LEDS; i++)
    {
        nrf_gpio_pin_clear(dcu_led_list[i]);
    }

}

bool first_connection = 1;


void DCU_set_connection_leds(uint16_t conn_handle, uint8_t state)
{
    ret_code_t err_code;

    uint32_t connections = usr_ble_get_conn_handle_len();

    if(connections == 0)
    {
        dcu_leds_reset();

        err_code = app_timer_start(indication_led_timer, APP_TIMER_TICKS(100), NULL);
        APP_ERROR_CHECK(err_code);

        first_connection = 1;

    }else{
        err_code = app_timer_stop(indication_led_timer);
        APP_ERROR_CHECK(err_code);

        if(first_connection) 
        {
            dcu_leds_reset();
            first_connection = 0;
        }
    }


    dcu_leds_t conn_led;
    conn_led = conn_handle + 11;
    
    switch (state)
    {
        case CONNECTION:
        {
            nrf_gpio_pin_set(conn_led);
        }
        break;

        case DISCONNECTION:
        {
            nrf_gpio_pin_clear(conn_led);
        }
        break;

        default:
        {
            nrf_gpio_pin_clear(conn_led);
        }
    }

}


#define RESET_REASON_HW_RESET   1
#define RESET_REASON_SW_RESET   4

void check_reset_reason()
{

#ifdef DEBUG
    // 1 -> HW reset
    // 4 -> Software reset
    uint32_t reset_reason = NRF_POWER->RESETREAS;
    NRF_LOG_INFO("Reset: %d", reset_reason);

    NRF_POWER->RESETREAS = 0xffffffff;

    switch (reset_reason)
    {
    case RESET_REASON_HW_RESET:
        NRF_LOG_INFO("Reset reason HW reset.")
        break;
    
    case RESET_REASON_SW_RESET:
        NRF_LOG_INFO("Reset reason SW reset.")
        break;
    
    default:
        NRF_LOG_INFO("Other reset reason: %d", reset_reason);
        break;
    }

    NRF_LOG_FLUSH();
#endif
}

void clocks_start(void)
{

    // Start LFXO and wait for it to start
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);

    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

}


/**@brief Function for initializing the timer. */
void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


