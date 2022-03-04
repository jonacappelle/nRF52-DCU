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
 *         File: usr_leds.c
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: On-board LED drivers
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#include "usr_leds.h"

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
#include "app_timer.h"

#define PIN_CPU_ACTIVITY    19

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

void DCU_set_connection_leds(dcu_connected_devices_t evt[], uint8_t state)
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

    for(uint16_t i=0; i<NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if(evt[i].conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            nrf_gpio_pin_set(dcu_led_list[i]);
        }else{
            nrf_gpio_pin_clear(dcu_led_list[i]);
        }
    }

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

// TODO can be moved to "usr_util.h"
void check_cpu_activity()
{
    nrf_gpio_pin_toggle(PIN_CPU_ACTIVITY);
}
