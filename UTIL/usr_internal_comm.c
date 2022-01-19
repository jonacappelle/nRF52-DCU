#include "usr_internal_comm.h"

#include "usr_uart.h"
#include "usr_ble.h"
#include "usr_time_sync.h"

#include "internal_comm_protocol.h"


// Logging
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#define NRF_LOG_MODULE_NAME usr_internal_comm_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();



static void decode_meas(uint8_t data)
{
    switch (data)
    {
    case COMM_CMD_MEAS_RAW:
        NRF_LOG_INFO("COMM_CMD_MEAS_RAW");
        set_config_raw_enable(1);
        comm_send_ok();
        break;

    case COMM_CMD_MEAS_QUAT6:
        NRF_LOG_INFO("COMM_CMD_MEAS_QUAT6");
        set_config_quat6_enable(1);
        comm_send_ok();
        break;

    case COMM_CMD_MEAS_QUAT9:
        NRF_LOG_INFO("COMM_CMD_MEAS_QUAT9");
        set_config_quat9_enable(1);
        comm_send_ok();
        break;

    case COMM_CMD_MEAS_WOM:
        NRF_LOG_INFO("COMM_CMD_MEAS_WOM");
        set_config_wom_enable(1);
        comm_send_ok();
        break;

    default:
        break;
    }
}

static void decode_sync(uint8_t data)
{
    ret_code_t err_code;

    switch (data)
    {
    case COMM_CMD_START_SYNC:
        NRF_LOG_INFO("COMM_CMD_START_SYNC");
        sync_enable();
        // comm_send_ok();
        break;
    
    case COMM_CMD_STOP_SYNC:
        NRF_LOG_INFO("COMM_CMD_STOP_SYNC");
        sync_disable();
        // comm_send_ok();
        break;

    default:
        sync_disable();
        break;
    }
}

static void decode_frequency(uint8_t data)
{
    set_config_frequency(data);
}

void send_battery_voltages()
{
    // | START_BYTE | packet_len | command (DATA_BYTE) |  sensor_nr |  data_type | data | CS |
    // | ----------- |-----------|-----------|------------|-----------|----------------|---|
    // | 1 byte     | 1 byte     | 1 byte               | 1 byte    | 1 byte    | k bytes | 1 byte |

    ret_code_t err_code;

    BATTERY_ARRAY bat;
    uint32_t len;
    get_battery(&bat, &len);

    uint8_t data_out[USR_INTERNAL_COMM_MAX_LEN]; //64 bytes long is more than enough for a data packet
    uint32_t data_len;
    data_type_byte_t type_byte;
    command_byte_t command_byte;

    // Fill configuration bytes
    data_out[0] = START_BYTE;

    data_len = 0;
    // Length of frame
    data_len += OVERHEAD_BYTES-1;

    // Tell the receiver its data we're sending
    command_byte = CONFIG;

    // TODO match this to the MAC address - keep track of list of MAC addresses
    // uint8_t sensor_nr = 0;

    data_out[2] = command_byte;
    // data_out[3] = sensor_nr;
    data_out[3] = COMM_CMD_REQ_BATTERY_LEVEL;

    // Copy data to packet
    memcpy((data_out + PACKET_DATA_PLACEHOLDER-1), &bat, len);

    // Set data len
    data_len += len;
    data_out[1] = (uint8_t) data_len;

    // Checksum
    uint8_t cs = calculate_cs(data_out, &data_len);
    data_out[PACKET_DATA_PLACEHOLDER-1 + len] = cs;

    // check for buffer overflows
    check_buffer_overflow(&data_len);

    // Send over UART to STM32
    uart_queued_tx(data_out, &data_len);
    // NRF_LOG_INFO("Data send");  
}



void uart_send_conn_dev(dcu_connected_devices_t* dev, uint32_t len)
{
    // | START_BYTE | packet_len | command (DATA_BYTE)  |  data_type | data    |   CS    |
    // | ----------- |-----------|----------------------|------------|---------|---------|
    // | 1 byte     | 1 byte     | 1 byte               |   1 byte    | k bytes | 1 byte |

    ret_code_t err_code;

    uint8_t data_out[USR_INTERNAL_COMM_MAX_LEN]; //64 bytes long is more than enough for a data packet
    uint32_t data_len;
    data_type_byte_t type_byte;
    command_byte_t command_byte;

    // Fill configuration bytes
    data_out[0] = START_BYTE;

    data_len = 0;
    // Length of frame
    data_len += OVERHEAD_BYTES-1;

    // Tell the receiver its data we're sending
    command_byte = CONFIG;

    // TODO match this to the MAC address - keep track of list of MAC addresses
    // uint8_t sensor_nr = 0;

    data_out[2] = command_byte;
    // data_out[3] = sensor_nr;
    data_out[3] = COMM_CMD_REQ_CONN_DEV_LIST;

    // Copy data to packet
    memcpy((data_out + PACKET_DATA_PLACEHOLDER-1), dev, len);

    // Set data len
    data_len += len;
    data_out[1] = (uint8_t) data_len;

    // Checksum
    uint8_t cs = calculate_cs(data_out, &data_len);
    data_out[PACKET_DATA_PLACEHOLDER-1 + len] = cs;

    // check for buffer overflows
    check_buffer_overflow(&data_len);

    // Send over UART to STM32
    uart_queued_tx(data_out, &data_len);
    // NRF_LOG_INFO("Data send");   
}


void comm_rx_process(void *p_event_data, uint16_t event_size)
{
    //| START_BYTE | packet_len | command (CONFIG_BYTE) |  config_data | CS    |
    //| ----------- |-----------|---------           --|------------- |-----  -|
    //| 1 byte        | 1 byte  |               1 byte |      k bytes |  1 byte |


    // Get data from buffer - store in var rx_data
    uint8_t p_byte;
    uint8_t rx_data[USR_INTERNAL_COMM_MAX_LEN];
    uint8_t i = 0;
    while ((uart_rx_buff_get(&p_byte) != NRF_ERROR_NOT_FOUND))
    {
        rx_data[i] = p_byte;
        i++;
    }

    // Store the received packet length
    uint32_t len = i;

    // Check if start byte is correct
    if(rx_data[0] != START_BYTE)
    {
        NRF_LOG_INFO("Invalid RX packet received");
        NRF_LOG_INFO("Start byte not correct");
        return;
    }

    // Check if packet len == received packet len
    if(rx_data[1] != len)
    {
        NRF_LOG_INFO("Invalid RX packet received");
        NRF_LOG_INFO("Invalid RX packet len");
        return;
    }
    
    // Check checksum (not including last CS byte)
    uint32_t len_no_cs = len - CS_LEN;
    uint8_t cs = calculate_cs(rx_data, &len);

    if(cs != rx_data[len_no_cs])
    {
        NRF_LOG_INFO("Correct CS: 0x%X - Received CS: 0x%X", cs, rx_data[len_no_cs]);
        NRF_LOG_INFO("len: %d - 0x%X", len, len);

        NRF_LOG_INFO("Invalid RX packet received");
        NRF_LOG_INFO("Invalid RX packet CS");
        return;
    }

    // Check for command dataframe
    command_byte_t command = rx_data[2];
    if(command !=  CONFIG)
    {
        NRF_LOG_INFO("Invalid RX packet received");
        NRF_LOG_INFO("Invalid COMMAND received");
        return;
    }

    NRF_LOG_INFO("Correct payload packet received");
    NRF_LOG_FLUSH();

    // Decode payload
    uint8_t remaining_data_len = len_no_cs - CONFIG_PACKET_DATA_OFFSET;
    uint8_t j = CONFIG_PACKET_DATA_OFFSET;
    
    while(remaining_data_len >=1) // Keep processing when there is data available
    {
        NRF_LOG_INFO("remaining_data_len: %d", remaining_data_len);

        uint8_t config_data = rx_data[j]; // Get packet
        NRF_LOG_INFO("Config data: 0x%X", config_data);
        NRF_LOG_FLUSH();

        switch (config_data)
        {
        case COMM_CMD_SET_CONN_DEV_LIST: // WORKING
        {

            NRF_LOG_INFO("COMM_CMD_SET_CONN_DEV_LIST");
            
            dcu_conn_dev_t conn_dev[NRF_BLE_SCAN_ADDRESS_CNT];
             // Init with zeros
            memset(conn_dev, 0, sizeof(conn_dev));

            // Copy data into buffer
            memcpy(conn_dev, &rx_data[j+1], sizeof(conn_dev));

            // Return packet with connected devices
            set_conn_dev_mask(conn_dev, sizeof(conn_dev));

            remaining_data_len = remaining_data_len - sizeof(conn_dev) - 1;
            // j++;

            comm_send_ok();

        }break;

        case COMM_CMD_REQ_CONN_DEV_LIST: // WORKING
        {
            NRF_LOG_INFO("COMM_CMD_REQ_CONN_DEV_LIST");

            dcu_connected_devices_t dev[NRF_SDH_BLE_CENTRAL_LINK_COUNT];
            get_connected_devices(dev, sizeof(dev));
            
            uart_send_conn_dev(dev, sizeof(dev));

            remaining_data_len--;
            j++;

        } break;

        case COMM_CMD_START: // WORKING

            NRF_LOG_INFO("COMM_CMD_START");

            config_send();
            remaining_data_len--;
            j++;
            break;

        case COMM_CMD_STOP:

            NRF_LOG_INFO("COMM_CMD_STOP");

            config_send_stop();
            // set_config_reset();
            // config_send();
            remaining_data_len--;
            j++;
            break;

        case COMM_CMD_MEAS: // WORKING

            NRF_LOG_INFO("COMM_CMD_MEAS");

            // Check which measurement to start
            config_data = rx_data[j+1]; // Peek the next byte

            // Decode meas payload
            decode_meas(config_data);

            remaining_data_len = remaining_data_len-2;
            j=j+2;
            break;
        
        case COMM_CMD_SYNC: // WORKING

            NRF_LOG_INFO("COMM_CMD_SYNC");
            
            config_data = rx_data[j+1];
            
            decode_sync(config_data);

            remaining_data_len = remaining_data_len-2;
            j=j+2;
            break;

        case COMM_CMD_FREQUENCY: // WORKING

            NRF_LOG_INFO("COMM_CMD_FREQUENCY");

            config_data = rx_data[j+1];

            decode_frequency(config_data);
            comm_send_ok();

            remaining_data_len = remaining_data_len-2;
            j=j+2;
            break;

        case COMM_CMD_CALIBRATE:

            NRF_LOG_INFO("COMM_CMD_CALIBRATE");

            set_config_start_calibration(1);
            config_send();
            set_config_start_calibration(0);

            remaining_data_len--;
            j++;

            break;

        case COMM_CMD_RESET:

            NRF_LOG_INFO("COMM_CMD_RESET");

            set_config_reset();
            comm_send_ok();

            remaining_data_len--;
            j++;
            break;

        case COMM_CMD_REQ_BATTERY_LEVEL: // WORKING

            NRF_LOG_INFO("COMM_CMD_REQ_BATTERY_LEVEL");

            // TODO return battery level packet
            send_battery_voltages();

            remaining_data_len--;
            j++;
            break;

        default:
            break;
        }
    }

    check_not_negative_uint8(&remaining_data_len);
}


static uint8_t calculate_cs(uint8_t * data, uint32_t * len) //tested
{
    // Init to zero
    uint8_t cs = 0x00;

    for (uint32_t i=0; i<(*len-1); i++)
    {
        // XOR
        cs ^= data[i];
    }

    return cs;
}

void comm_send_ok()
{
    // | START_BYTE | packet_len | command (DATA_BYTE) |  sensor_nr |  data_type | data | CS |
    // | ----------- |-----------|-----------|------------|-----------|----------------|---|
    // | 1 byte     | 1 byte     | 1 byte               | 1 byte    | 1 byte    | k bytes | 1 byte |

    NRF_LOG_INFO("SEND FEEDBACK over uart");

    ret_code_t err_code;

    uint8_t data_out[USR_INTERNAL_COMM_MAX_LEN]; //64 bytes long is more than enough for a data packet
    uint32_t data_len;
    data_type_byte_t type_byte;
    command_byte_t command_byte;

    // Fill configuration bytes
    data_out[0] = START_BYTE;

    data_len = 0;
    // Length of frame
    data_len += OVERHEAD_BYTES;

    // Tell the receiver its config we're sending
    command_byte = CONFIG;

    // TODO match this to the MAC address - keep track of list of MAC addresses
    // uint8_t sensor_nr = data_in->conn_handle + 1;
    uint8_t sensor_nr = 0xFF;

    data_out[2] = command_byte;
    data_out[3] = COMM_CMD_OK;
    data_out[4] = sensor_nr;

    // data_len += sizeof(uint8_t); // No data to be added to OK packet
    data_out[1] = (uint8_t) data_len;

    // Checksum
    uint8_t cs = calculate_cs(data_out, &data_len);
    data_out[PACKET_DATA_PLACEHOLDER] = cs;

    // check for buffer overflows
    check_buffer_overflow(&data_len);

    // Send over UART to STM32
    uart_queued_tx(data_out, &data_len);
}


// Tested and working
void comm_process(ble_imu_service_c_evt_type_t type, ble_imu_service_c_evt_t * data_in)
{
    // | START_BYTE | packet_len | command (DATA_BYTE) |  sensor_nr |  data_type | data | CS |
    // | ----------- |-----------|-----------|------------|-----------|----------------|---|
    // | 1 byte     | 1 byte     | 1 byte               | 1 byte    | 1 byte    | k bytes | 1 byte |

    ret_code_t err_code;

    uint8_t data_out[USR_INTERNAL_COMM_MAX_LEN]; //64 bytes long is more than enough for a data packet
    uint32_t data_len;
    data_type_byte_t type_byte;
    command_byte_t command_byte;

    // Fill configuration bytes
    data_out[0] = START_BYTE;

    // Calibration info
    if(type == BLE_IMU_SERVICE_EVT_INFO) // If we have calibration info, send it!
    {
        NRF_LOG_INFO("SEND CALIBRATION CONFIG over uart");

        data_len = 0;
        // Length of frame
        data_len += OVERHEAD_BYTES;

        // Tell the receiver its config we're sending
        command_byte = CONFIG;

        // TODO match this to the MAC address - keep track of list of MAC addresses
        uint8_t sensor_nr = data_in->conn_handle;

        data_out[2] = command_byte;
        
        data_out[4] = sensor_nr;

        // NRF_LOG_HEXDUMP_INFO(data_out, data_len);

        uint8_t temp;

        if(data_in->params.value.info_data.sync_complete || data_in->params.value.info_data.sync_lost)
        {
            if(data_in->params.value.info_data.sync_complete) temp = COMM_CMD_SYNC_COMPLETE;
            else if(data_in->params.value.info_data.sync_lost) temp = COMM_CMD_SYNC_LOST;

            data_out[3] = COMM_CMD_SYNC;

        }else{
            if(data_in->params.value.info_data.calibration_start) temp = COMM_CMD_CALIBRATION_START;
            
            if(data_in->params.value.info_data.gyro_calibration_done) temp = COMM_CMD_CALIBRATION_GYRO_DONE;
            
            if (data_in->params.value.info_data.accel_calibration_drone && data_in->params.value.info_data.gyro_calibration_done) temp = COMM_CMD_CALIBRATION_ACCEL_DONE;
            
            if(data_in->params.value.info_data.accel_calibration_drone && data_in->params.value.info_data.gyro_calibration_done && data_in->params.value.info_data.mag_calibration_done) temp = COMM_CMD_CALIBRATION_MAG_DONE;

            if(data_in->params.value.info_data.calibration_done) temp = COMM_CMD_CALIBRATION_DONE;

            data_out[3] = COMM_CMD_CALIBRATE;
        }


        data_len += sizeof(uint8_t);
        data_out[1] = (uint8_t) data_len;

        // Copy data into packet
        memcpy((data_out + PACKET_DATA_PLACEHOLDER), &temp, sizeof(temp));

        // NRF_LOG_HEXDUMP_INFO(data_out, data_len);

        // Checksum
        uint8_t cs = calculate_cs(data_out, &data_len);
        data_out[PACKET_DATA_PLACEHOLDER + sizeof(temp)] = cs;

        // NRF_LOG_HEXDUMP_INFO(data_out, data_len);

        // Send over UART to STM32
        uart_queued_tx(data_out, &data_len);
        // NRF_LOG_INFO("Data send");

    }else{ // Else it's DATA

    // BLE_PACKET_BUFFER_COUNT bytes in 1 BLE packet
    for(uint8_t i=0; i<BLE_PACKET_BUFFER_COUNT; i++)
    {
        data_len = 0;
        // Length of frame
        data_len += OVERHEAD_BYTES;

        // Tell the receiver its data we're sending
        command_byte = DATA;

        uint8_t sensor_nr = data_in->conn_handle;

        data_out[2] = command_byte;
        data_out[3] = sensor_nr;

        switch (type)
        {
            case BLE_IMU_SERVICE_EVT_QUAT:
            {
                ble_imu_service_quat_t *quat = &data_in->params.value.quat_data;

                data_len += (4*sizeof(int32_t))/sizeof(uint8_t);
                data_out[1] = (uint8_t) data_len; //22

                type_byte = QUATERNIONS;
                data_out[4] = type_byte;

                // Copy data to packet
                memcpy((data_out + PACKET_DATA_PLACEHOLDER), &quat->quat[i].w, sizeof(int32_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + sizeof(int32_t)), &quat->quat[i].x, sizeof(int32_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 2*sizeof(int32_t)), &quat->quat[i].y, sizeof(int32_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 3*sizeof(int32_t)), &quat->quat[i].z, sizeof(int32_t));

                // Checksum
                uint8_t cs = calculate_cs(data_out, &data_len);
                data_out[PACKET_DATA_PLACEHOLDER + 4*sizeof(int32_t)] = cs;

            }break;

            case BLE_IMU_SERVICE_EVT_EULER:
            {
                NRF_LOG_INFO("Not implemented yet");
                
                
            }break;
            case BLE_IMU_SERVICE_EVT_RAW:
            {
                ble_imu_service_raw_t *raw = &data_in->params.value.raw_data;

                data_len += (3*3*sizeof(int16_t))/sizeof(uint8_t);

                type_byte = RAW;
                data_out[4] = type_byte;
                data_out[1] = (uint8_t) data_len; //24 bytes

                // Copy data to packet
                memcpy((data_out + PACKET_DATA_PLACEHOLDER), &raw->single_raw[i].accel.x, sizeof(int16_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + sizeof(int16_t)), &raw->single_raw[i].accel.y, sizeof(int16_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 2*sizeof(int16_t)), &raw->single_raw[i].accel.z, sizeof(int16_t));

                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 3*sizeof(int16_t)), &raw->single_raw[i].gyro.x, sizeof(int16_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 4*sizeof(int16_t)), &raw->single_raw[i].gyro.y, sizeof(int16_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 5*sizeof(int16_t)), &raw->single_raw[i].gyro.z, sizeof(int16_t));

                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 6*sizeof(int16_t)), &raw->single_raw[i].compass.x, sizeof(int16_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 7*sizeof(int16_t)), &raw->single_raw[i].compass.y, sizeof(int16_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 8*sizeof(int16_t)), &raw->single_raw[i].compass.z, sizeof(int16_t));

                // Checksum
                uint8_t cs = calculate_cs(data_out, &data_len);
                data_out[PACKET_DATA_PLACEHOLDER + 3*3*sizeof(int16_t)] = cs;

            }break;

            default:
            {

                NRF_LOG_INFO("default");
                NRF_LOG_FLUSH();

            }break;
        }

        // check for buffer overflows
        check_buffer_overflow(&data_len);

        // Send over UART to STM32
        uart_queued_tx(data_out, &data_len);
        // NRF_LOG_INFO("Data send");

    }

    }
}



// Error checking helper functions

static void check_buffer_overflow(uint32_t* len)
{
    ret_code_t err_code;

    if(*len >= USR_INTERNAL_COMM_MAX_LEN)
    {
        NRF_LOG_INFO("data_len: %d", *len);
        err_code = NRF_ERROR_NO_MEM;
        APP_ERROR_CHECK(err_code);
    }

}


static void check_not_negative_uint8(uint8_t* data)
{
    ret_code_t err_code;

    if(*data < 0)
    {
        err_code = NRF_ERROR_INVALID_LENGTH;
        APP_ERROR_CHECK(err_code);
    }
}