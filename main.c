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




int main(void)
{
    ret_code_t err_code;

    nrf_delay_ms(2000);

    // Initialize.
    log_init();

    // 1 -> HW reset
    // 4 -> Software reset
    uint32_t reset_reason = NRF_POWER->RESETREAS;
    NRF_LOG_INFO("Reset: %d", reset_reason);
    NRF_LOG_FLUSH();

    NRF_POWER->RESETREAS = 0xffffffff;


    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    // dfu_async_init();

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
    NRF_LOG_FLUSH();

    #if USR_ADVERTISING == 1
    gap_params_init();
    #endif
    
    // Not needed - by setting correct settings in sdk_config.h this will be enabled
    // conn_evt_len_ext_set(); // added for faster speed

    gatt_init();

    #if USR_ADVERTISING == 1
    advertising_init();
    #endif

    // Initialize BLE services
    services_init();

    #if USR_ADVERTISING == 1
    conn_params_init();
    #endif

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
    sync_timer_init();

    // Initialize pins for debugging
    usr_gpio_init();

    // leds_startup();
    
    create_timers(); // Needs to be places after softdevice initialization

    #if USR_ADVERTISING == 1
    advertising_start(false);
    #endif

    nrf_gpio_cfg_output(TIMESYNC_PIN);

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
