#include "usr_internal_comm.h"

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
    CMD_LIST_CONN_DEV = 1,
    CMD_GYRO,
    CMD_ACCEL,
    CMD_MAG,
    CMD_EULER,
    CMD_QUAT6,
    CMD_QUAT9,
    CMD_START_SYNC,
    CMD_STOP_SYNC,
    CMD_SET_FREQUENCY,
    CMD_RESET,
    CMD_SEND_CONFIG,
    CMD_BATTERY_LEVEL
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


void comm_parse_quat(uint8_t sensor_nr, int16_t w, int16_t x, int16_t y, int16_t z, uint8_t *data, uint32_t * len) // tested
{
    // | START_BYTE | packet_len | command (DATA_BYTE) |  sensor_nr |  data_type | data | CS |
    // | ----------- |-----------|-----------|------------|-----------|----------------|---|
    // | 1 byte     | 1 byte     | 1 byte               | 1 byte    | 1 byte    | k bytes | 1 byte |


    uint32_t data_len = (sizeof(w) + sizeof(x) + sizeof(y) + sizeof(z))/sizeof(uint8_t);
    *len = data_len + OVERHEAD_BYTES;

    command_byte_t command_byte = DATA;
    data_type_byte_t type_byte = QUATERNIONS;

    data[0] = START_BYTE;
    data[1] = (uint8_t) *len;
    data[2] = command_byte;
    data[3] = sensor_nr;
    data[4] = type_byte;

    // // Copy data to packet
    memcpy((data + PACKET_DATA_PLACEHOLDER), &w, sizeof(w));
    memcpy((data + PACKET_DATA_PLACEHOLDER + sizeof(w)), &x, sizeof(x));
    memcpy((data + PACKET_DATA_PLACEHOLDER + 2*sizeof(w)), &y, sizeof(y));
    memcpy((data + PACKET_DATA_PLACEHOLDER + 3*sizeof(w)), &z, sizeof(z));

    uint8_t cs = calculate_cs(data, len);
    data[PACKET_DATA_PLACEHOLDER + 4*sizeof(w)] = cs;

    // NRF_LOG_INFO("QUAT parsed");
}




