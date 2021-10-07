#include "usr_internal_comm.h"

#include "usr_uart.h"

// Logging
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#define NRF_LOG_MODULE_NAME usr_internal_comm_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#define START_BYTE          0x73 // s
#define OVERHEAD_BYTES      6
#define PACKET_DATA_PLACEHOLDER 5


typedef enum 
{ 
    DATA = 1,
    CONFIG
} command_byte_t;

typedef enum 
{ 
    QUATERNIONS = 1,
    EULER,
    RAW
} data_type_byte_t;

typedef enum 
{ 
    COMM_CMD_LIST_CONN_DEV = 1,
    COMM_CMD_GYRO,
    COMM_CMD_ACCEL,
    COMM_CMD_MAG,
    COMM_CMD_EULER,
    COMM_CMD_QUAT6,
    COMM_CMD_QUAT9,
    COMM_CMD_START_SYNC,
    COMM_CMD_STOP_SYNC,
    COMM_CMD_SET_FREQUENCY,
    COMM_CMD_RESET,
    COMM_CMD_SEND_CONFIG,
    COMM_CMD_BATTERY_LEVEL
} command_type_byte_t;



uint8_t calculate_cs(uint8_t * data, uint32_t * len) //tested
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


void comm_process(ble_imu_service_c_evt_type_t type, ble_imu_service_c_evt_t * data_in)
{
    // | START_BYTE | packet_len | command (DATA_BYTE) |  sensor_nr |  data_type | data | CS |
    // | ----------- |-----------|-----------|------------|-----------|----------------|---|
    // | 1 byte     | 1 byte     | 1 byte               | 1 byte    | 1 byte    | k bytes | 1 byte |

    NRF_LOG_INFO("comm_process starting...");
    NRF_LOG_FLUSH();

    ble_imu_service_quat_t *quat = &data_in->params.value.quat_data;

    // BLE_PACKET_BUFFER_COUNT bytes in 1 BLE packet
    for(uint8_t i=0; i<BLE_PACKET_BUFFER_COUNT; i++)
    {

        uint8_t data_out[64];

        uint32_t data_len = 0;
        data_type_byte_t type_byte;

        switch (type)
        {
            case BLE_IMU_SERVICE_EVT_QUAT:
            {
                NRF_LOG_INFO("comm_process evt_quat");
                NRF_LOG_FLUSH();

                data_len += (4*sizeof(int32_t))/sizeof(uint8_t);

                type_byte = QUATERNIONS;

                // Copy data to packet
                memcpy((data_out + PACKET_DATA_PLACEHOLDER), &data_in->params.value.quat_data.quat[i].w, sizeof(int32_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + sizeof(int32_t)), &quat->quat[i].x, sizeof(int32_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 2*sizeof(int32_t)), &quat->quat[i].y, sizeof(int32_t));
                memcpy((data_out + PACKET_DATA_PLACEHOLDER + 3*sizeof(int32_t)), &quat->quat[i].z, sizeof(int32_t));

            }break;

            case BLE_IMU_SERVICE_EVT_EULER:
            {
                
            }break;
            case BLE_IMU_SERVICE_EVT_RAW:
            {

            }break;

            default:
            {

                NRF_LOG_INFO("default");
                NRF_LOG_FLUSH();

            }break;
        }

        // Length of frame
        data_len += OVERHEAD_BYTES;

        command_byte_t command_byte = DATA;

        uint8_t sensor_nr = data_in->conn_handle + 1;

        // Fill configuration bytes
        data_out[0] = START_BYTE;
        data_out[1] = (uint8_t) data_len;
        data_out[2] = command_byte;
        data_out[3] = sensor_nr;
        data_out[4] = type_byte;

        NRF_LOG_INFO("data_len: %d", data_len);

        // Checksum
        uint8_t cs = calculate_cs(data_out, &data_len);
        data_out[PACKET_DATA_PLACEHOLDER + 4*sizeof(int32_t)] = cs;

        NRF_LOG_INFO("cs calculated");
        NRF_LOG_FLUSH();

        // Send over UART to STM32
        uart_queued_tx(data_out, &data_len);
        NRF_LOG_INFO("QUAT send");

    }
}




