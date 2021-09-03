#include "usr_uart.h"

// Includes for libuarte
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_libuarte_async.h"
#include "nrf_drv_clock.h"
#include <bsp.h>
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_queue.h"

// Buffers
#include "app_fifo.h"

// Application scheduler
#include "app_scheduler.h"

// Logging
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


//////////////////////////////////////
// Resources used:
//                - UART0
//                - Timer1
//                - RTC2
//////////////////////////////////////


// Define LIBUARTE instance
//(_name, _uarte_idx, _timer0_idx, _rtc1_idx, _timer1_idx, _rx_buf_size, _rx_buf_cnt)
NRF_LIBUARTE_ASYNC_DEFINE(libuarte, 0, 1, 2, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 255, 3);


typedef struct uart
{
    // Keep track if UART transaction is ongoing
    bool in_progress;
    // Event scheduled handler
    app_sched_event_handler_t uart_scheduled;
} uart_t;

// Initialize uart variables
static uart_t uart = {
    .in_progress = 0,
};

// Buffer for UART
typedef struct uart_buffer
{
    // Instance of FIFO buffer for transmitted bytes
    app_fifo_t uart_tx_buff_instance;
    uint8_t tx_buff[1024];
    // Static temp buffer for transmitting bytes (not used by app_fifo) - static memory because libuarte tx requires this
    uint8_t uart_tx_buff[512];
    uint8_t uart_tx_done_buff[1024];
    // Length of static temp buffer for transmitting bytes
    uint32_t uart_tx_buff_len;
    uint32_t uart_tx_done_buff_len;
    // Instance of FIFO buffer for received bytes
    app_fifo_t uart_rx_buff_instance;
    // RX buffer
    uint8_t rx_buff[256];
} uart_buffer_t;

// Initialization of uart buffer
static uart_buffer_t buffer;


static void uart_buffer_init()
{
    ret_code_t err_code;

    // Initialize FIFO for TX bytes
    err_code = app_fifo_init(&buffer.uart_tx_buff_instance, buffer.tx_buff, (uint16_t)sizeof(buffer.tx_buff));
    APP_ERROR_CHECK(err_code);

    // Initialize FIFO for RX bytes
    err_code = app_fifo_init(&buffer.uart_rx_buff_instance, buffer.rx_buff, (uint16_t)sizeof(buffer.rx_buff));
    APP_ERROR_CHECK(err_code);
}

bool uart_in_progress()
{
    return uart.in_progress;
}

void uart_print(char msg[])
{
    ret_code_t err_code;

    uint32_t len[1];
    len[0] = strlen(msg);

    uint8_t data[256];
    memcpy(data, msg, len[0]);

    uart_queued_tx(data, len);
}


uint32_t uart_rx_to_cmd(uint8_t *command_in, uint8_t len)
{
    uint8_t temp[len];
    memcpy(temp, command_in, len);

    NRF_LOG_INFO("%d %d %d", command_in[0], command_in[1], command_in[2]);
    // NRF_LOG_FLUSH();

    uint32_t x = atoi(temp);

    NRF_LOG_INFO("%d", x);
    // NRF_LOG_FLUSH();

    return x;
}


void uart_event_handler(void * context, nrf_libuarte_async_evt_t * p_evt)
{
    nrf_libuarte_async_t * p_libuarte = (nrf_libuarte_async_t *)context;
    ret_code_t err_code;
    uint16_t index = 0;

    switch (p_evt->type)
    {
        case NRF_LIBUARTE_ASYNC_EVT_ERROR:

            NRF_LOG_INFO("NRF_LIBUARTE_ASYNC_EVT_ERROR");

            break;

        case NRF_LIBUARTE_ASYNC_EVT_OVERRUN_ERROR:

            NRF_LOG_INFO("NRF_LIBUARTE_ASYNC_EVT_OVERRUN_ERROR");

            break;

        case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:

            NRF_LOG_INFO("NRF_LIBUARTE_ASYNC_EVT_RX_DATA");

            if(p_evt->data.rxtx.length > 2)
            {
                NRF_LOG_ERROR("Error");
            }

            err_code = app_fifo_write(&buffer.uart_rx_buff_instance, p_evt->data.rxtx.p_data, (uint32_t *) &p_evt->data.rxtx.length);
            APP_ERROR_CHECK(err_code);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_INFO("app_fifo_put in NRF_DRV_UART_EVT_RX_DONE failed %d", err_code);
                NRF_LOG_FLUSH();
            }
            

            NRF_LOG_INFO("data: %d - %s", p_evt->data.rxtx.p_data[0], p_evt->data.rxtx.p_data[0]);

            NRF_LOG_INFO("FIFO put");

            err_code = app_sched_event_put(0, 0, uart.uart_scheduled);
            APP_ERROR_CHECK(err_code);

            // Free RX memory: if not done correctly can cause memory overflows
            nrf_libuarte_async_rx_free(p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);

            break;
        case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
        {
            // NRF_LOG_INFO("TX_DONE - start");

            // NRF_LOG_INFO("%s", buffer.uart_tx_done_buff);

            uint32_t index = 1024;

            // NRF_LOG_INFO("buffer.uart_tx_buff changed in NRF_LIBUARTE_ASYNC_EVT_TX_DONE");

            // Get next bytes from FIFO.
            err_code = app_fifo_read(&buffer.uart_tx_buff_instance, buffer.uart_tx_done_buff, &index);
            if (err_code == NRF_SUCCESS)
            {
                buffer.uart_tx_done_buff_len = index;
                // NRF_LOG_INFO("extra bytes1: %d", buffer.uart_tx_done_buff_len);

                err_code = nrf_libuarte_async_tx(&libuarte, buffer.uart_tx_done_buff, buffer.uart_tx_done_buff_len);
                APP_ERROR_CHECK(err_code);
                // NRF_LOG_INFO("Send next bytes from fifo");
            }else if (err_code = NRF_ERROR_NOT_FOUND) // FIFO is empty
            {
                // NRF_LOG_INFO("No data left in uart_tx_done_buff_instance buffer");

                // Notify TX done
                uart.in_progress = 0;
            }else{
                APP_ERROR_CHECK(err_code);
            }

            // NRF_LOG_INFO("TX_DONE - stop");
        }
            break;
        default:
            break;
    }
}

static void libuarte_clocks_init()
{
    // Setup necessary clocks - only necessary when not using softdevice
    #if SOFTDEVICE_ENABLED == 0
        ret_code_t ret = nrf_drv_clock_init();
        APP_ERROR_CHECK(ret);

        nrf_drv_clock_lfclk_request(NULL);
    #endif

}

void libuarte_init(app_sched_event_handler_t scheduled_function)
{
    ret_code_t err_code;

    libuarte_clocks_init();

    // Initialize FIFO buffers
    uart_buffer_init();

    // Pass scheduled callback function to uart library
    uart.uart_scheduled = scheduled_function;

    // Init params libuarte
    nrf_libuarte_async_config_t nrf_libuarte_async_config = {
            .tx_pin     = TX_PIN_NUMBER,
            .rx_pin     = RX_PIN_NUMBER,
            .baudrate   = NRF_UARTE_BAUDRATE_1000000, //NRF_UARTE_BAUDRATE_115200,
            .parity     = NRF_UARTE_PARITY_EXCLUDED,
            .hwfc       = NRF_UARTE_HWFC_ENABLED, // Yes, please !
            .timeout_us = 1000, //100,
            .int_prio   = APP_IRQ_PRIORITY_LOW // Higher interrupt priority than APP TIMER // APP_IRQ_PRIORITY_LOW
    };

    err_code = nrf_libuarte_async_init(&libuarte, &nrf_libuarte_async_config, uart_event_handler, (void *)&libuarte);
    APP_ERROR_CHECK(err_code);

    // Enable RX
    nrf_libuarte_async_enable(&libuarte);

    static uint8_t text[] = "ble_app_libUARTE example started.\r\n";
    static uint8_t text_size = sizeof(text);

    err_code = nrf_libuarte_async_tx(&libuarte, text, text_size);
    APP_ERROR_CHECK(err_code);
}


void uart_tx(uint8_t * p_data, size_t length)
{
    ret_code_t err_code;
    
    // Check data length: in libuarte implementation datalength may be larger
    if(length >= 255) NRF_LOG_INFO("Generated string too long! (%d bytes)", length);

    // Keep track of transaction started
    uart.in_progress = 1;

    // NRF_LOG_INFO("transmit");

    // Send bytes over UART (using DMA)
    err_code = nrf_libuarte_async_tx(&libuarte, p_data, length);
    APP_ERROR_CHECK(err_code);
}

void uart_queued_tx(uint8_t * data, uint32_t * len)
{
    // NRF_LOG_INFO("uart_queued_tx - start");

    ret_code_t err_code;
    uint32_t string_len;

    memcpy(&string_len, len, sizeof(string_len));

    // Put the data in FIFO buffer
    err_code = app_fifo_write(&buffer.uart_tx_buff_instance, data, len);
    APP_ERROR_CHECK(err_code);

    if (err_code == NRF_ERROR_NO_MEM)
    {
        NRF_LOG_INFO("UART FIFO BUFFER FULL!");
    }

    
    if (err_code == NRF_SUCCESS)
    {
        // The new byte has been added to FIFO. It will be picked up from there
        // (in 'uart_event_handler') when all preceding bytes are transmitted.
        // But if UART is not transmitting anything at the moment, we must start
        // a new transmission here.
        if (!uart.in_progress) // If UART TX transfer is not going on
        {
            // NRF_LOG_INFO("uart_queued_tx - uart not in progress");
            // This operation should be almost always successful, since we've
            // just added a byte to FIFO, but if some bigger delay occurred
            // (some heavy interrupt handler routine has been executed) since
            // that time, FIFO might be empty already.

            // NRF_LOG_INFO("buffer.uart_tx_buff changed in uart_queued_tx");

            err_code = app_fifo_read(&buffer.uart_tx_buff_instance, buffer.uart_tx_buff, &string_len);
            if (err_code == NRF_SUCCESS)
            {
                buffer.uart_tx_buff_len = string_len;
                // NRF_LOG_INFO("uart tx len %d", buffer.uart_tx_buff_len);

                // Transmit over uart
                uart_tx(buffer.uart_tx_buff, buffer.uart_tx_buff_len);
            }else
            {
                APP_ERROR_CHECK(err_code);
            }
        }
    }
}

ret_code_t uart_rx_buff_read(uint8_t * p_byte_array, uint32_t * p_size)
{
    ret_code_t err_code;

    // Read (p_size) bytes from RX FIFO buffer
    err_code = app_fifo_read(&buffer.uart_rx_buff_instance, p_byte_array, p_size);

    // Return in case of an error
    return err_code;
}

ret_code_t uart_rx_buff_get(uint8_t * p_byte)
{
    ret_code_t err_code;

    // Read 1 byte from RX FIFO buffer
    err_code = app_fifo_get(&buffer.uart_rx_buff_instance, p_byte);

    // Return in case of an error
    return err_code;
}
