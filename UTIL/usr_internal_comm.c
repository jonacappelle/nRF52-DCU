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

    

    // BLE_PACKET_BUFFER_COUNT bytes in 1 BLE packet
    for(uint8_t i=0; i<BLE_PACKET_BUFFER_COUNT; i++)
    {

        uint8_t data_out[64]; //64 bytes long is more than enough for a data packet

        uint32_t data_len = 0;
        data_type_byte_t type_byte;

                // Length of frame
        data_len += OVERHEAD_BYTES;

        command_byte_t command_byte = DATA;

        uint8_t sensor_nr = data_in->conn_handle + 1;

        // Fill configuration bytes
        data_out[0] = START_BYTE;
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

        // TODO change place for CS - is now at wrong place

        // Send over UART to STM32
        uart_queued_tx(data_out, &data_len);
        // NRF_LOG_INFO("Data send");

    }
}




