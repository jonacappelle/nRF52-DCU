#include "usr_time_sync.h"

// #include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"

// Time Synchronization
#include "time_sync.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"

#include "nordic_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "boards.h"

static uint32_t ts_pin = 0;
static uint32_t ts_meas_pin = 0;
static bool m_imu_trigger_enabled = 0;

// Function to enable triggering of sync pin
void ts_imu_trigger_enable(void)
{
    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_target;
    ret_code_t err_code;

    if (m_imu_trigger_enabled)
    {
        return;
    }

    // Round up to nearest second to next 2000 ms to start toggling.
    // If the receiver has received a valid sync packet within this time, the GPIO toggling polarity will be the same.

    time_now_ticks = ts_timestamp_get_ticks_u64();
    time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

    time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec) + (1000 * 2);
    time_target = (time_target / 1000) * 1000;

    err_code = ts_set_trigger(time_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
    APP_ERROR_CHECK(err_code);

    nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);

    m_imu_trigger_enabled = 1;
}

void ts_imu_trigger_disable(void)
{
    m_imu_trigger_enabled = 0;
}

bool ts_get_imu_trigger_enabled(void)
{
    return m_imu_trigger_enabled;
}


void timesync_pin_toggle(uint32_t tick)
{
    // Toggle on multiples of 100 ticks
    if( (tick % 1000) == 0)
    {
        nrf_gpio_pin_toggle(ts_pin);
    }
}


// Timesync event handler
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

            tick_target = evt->params.triggered.tick_target + 10;

            // NRF_LOG_INFO("tick_target %d", tick_target);

            ret_code_t err_code = ts_set_trigger(tick_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));

            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("ts_evt_callback ERROR: %d", err_code);
                NRF_LOG_FLUSH();
            }
            APP_ERROR_CHECK(err_code);

            // Toggle LED to measure TimeSync
            timesync_pin_toggle(tick_target);
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


void timesync_pin_init(const usr_timesync_config_t * cfg)
{
    // Configure local variables based on config
    ts_pin = cfg->ts_led_pin;
    ts_meas_pin = cfg->ts_meas_pin;

    // Config output pin for visual feedback
    nrf_gpio_cfg_output(ts_pin);

        // Config debug pin:
        // nRF52-DK (PCA10040) Toggle P0.24 from sync timer to allow pin measurement
        // nRF52840-DK (PCA10056) Toggle P1.14 from sync timer to allow pin measurement
    #if defined(BOARD_PCA10040)
        nrf_gpiote_task_configure(3, ts_meas_pin, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
        // nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, 24), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
        nrf_gpiote_task_enable(3);
    #elif defined(BOARD_PCA10056)
        // nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 14), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
        nrf_gpiote_task_configure(3, ts_meas_pin, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
        nrf_gpiote_task_enable(3);
    #elif defined(BOARD_CUSTOM)
        // nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, 24), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
        nrf_gpiote_task_configure(3, ts_meas_pin, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
        nrf_gpiote_task_enable(3);
    #else
    #warning Debug pin not set
    #endif
}


void sync_timer_init()
{
    ret_code_t err_code;

    usr_timesync_config_t usr_ts_config =
    {
        .ts_led_pin = 13,
        .ts_meas_pin = 24
    };

    timesync_pin_init(&usr_ts_config);

    ts_init_t init_ts =
        {
            .high_freq_timer[0] = NRF_TIMER3,
            .high_freq_timer[1] = NRF_TIMER4,
            .egu = NRF_EGU3,
            .egu_irq_type = SWI3_EGU3_IRQn,
            .evt_handler = ts_evt_callback,
        };

    err_code = ts_init(&init_ts);
    APP_ERROR_CHECK(err_code);

    ts_rf_config_t rf_config =
        {
            .rf_chn = 80,
            .rf_addr = {0xDE, 0xAD, 0xBE, 0xEF, 0x19}};

    err_code = ts_enable(&rf_config);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Started listening for beacons.\r\n");
    NRF_LOG_INFO("Press Button 1 to start transmitting sync beacons\r\n");
    NRF_LOG_INFO("GPIO toggling will begin when transmission has started.\r\n");
}


void ts_print_sync_time()
{
    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_ticks;

    time_now_ticks = ts_timestamp_get_ticks_u64();
    time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;
    time_ticks = TIME_SYNC_MSEC_TO_TICK(time_now_msec);
    NRF_LOG_INFO("Time: ticks %d - ms %d", time_ticks, time_now_msec);
}

uint64_t usr_ts_timestamp_get_ticks_u64()
{
    return ts_timestamp_get_ticks_u64;
}