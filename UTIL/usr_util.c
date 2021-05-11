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




void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
     /* static error variables - in order to prevent removal by optimizers */
    static volatile struct
    {
        uint32_t        fault_id;
        uint32_t        pc;
        uint32_t        error_info;
        assert_info_t * p_assert_info;
        error_info_t  * p_error_info;
        ret_code_t      err_code;
        uint32_t        line_num;
        const uint8_t * p_file_name;
    } m_error_data = {0};

    // The following variable helps Keil keep the call stack visible, in addition, it can be set to
    // 0 in the debugger to continue executing code after the error check.
    volatile bool loop = true;
    UNUSED_VARIABLE(loop);

    m_error_data.fault_id   = id;
    m_error_data.pc         = pc;
    m_error_data.error_info = info;

    switch (id)
    {
        case NRF_FAULT_ID_SDK_ASSERT:
            m_error_data.p_assert_info = (assert_info_t *)info;
            m_error_data.line_num      = m_error_data.p_assert_info->line_num;
            m_error_data.p_file_name   = m_error_data.p_assert_info->p_file_name;
            break;

        case NRF_FAULT_ID_SDK_ERROR:
            m_error_data.p_error_info = (error_info_t *)info;
            m_error_data.err_code     = m_error_data.p_error_info->err_code;
            m_error_data.line_num     = m_error_data.p_error_info->line_num;
            m_error_data.p_file_name  = m_error_data.p_error_info->p_file_name;
            break;
    }

    UNUSED_VARIABLE(m_error_data);

    // If printing is disrupted, remove the irq calls, or set the loop variable to 0 in the debugger.
    __disable_irq();

    NRF_LOG_INFO("err_code: %d", m_error_data.err_code);
    NRF_LOG_INFO("line_num: %d", m_error_data.line_num);
    NRF_LOG_INFO("p_file_name: %s", m_error_data.p_file_name);
    NRF_LOG_FINAL_FLUSH();

    while (loop);

    __enable_irq();
}