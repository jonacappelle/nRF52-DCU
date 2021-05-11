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



void usr_gpio_init()
{
    // Check time needed to process data
    nrf_gpio_cfg_output(18);
    // Check active time of CPU
    // nrf_gpio_cfg_output(19);
    // Check time of NRF_LOG_FLUSH
    nrf_gpio_cfg_output(20);
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



