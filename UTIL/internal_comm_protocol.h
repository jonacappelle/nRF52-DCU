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
 *         File: internal_comm_protocol.h
 *      Created: 14-5-2021
 *       Author: Jona Cappelle
 *      Version: v0.1
 *
 *  Description: Communication between nRF52 & STM32
 *               STM32 implementation - PROTOCOL definitions
 *
 *  Commissiond by the Interreg NOMADe project
 * 
 */

#ifndef _USR_INTERNAL_COMM_PROTOCOL_H__
#define _USR_INTERNAL_COMM_PROTOCOL_H__

//  ____________________________________________________________________________________________
// | START_BYTE  | packet_len | command (DATA_BYTE) |  sensor_nr | data_type | data    | CS     |
// | ----------- |----------- |-----------          |------------|-----------|---------|------- |
// | 1 byte      | 1 byte     | 1 byte              | 1 byte     | 1 byte    | k bytes | 1 byte |
//  ____________________________________________________________________________________________


#define START_BYTE                      0x73 // s
#define OVERHEAD_BYTES                  6
#define PACKET_DATA_PLACEHOLDER         5
#define USR_INTERNAL_COMM_MAX_LEN       128
#define CONFIG_PACKET_DATA_OFFSET       3
#define CS_LEN                          1


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
    COMM_CMD_MEAS_RAW = 1,
    COMM_CMD_MEAS_QUAT6,
    COMM_CMD_MEAS_QUAT9,
    COMM_CMD_MEAS_WOM
} command_type_meas_byte_t;

typedef enum
{
    COMM_CMD_START_SYNC = 1,
    COMM_CMD_STOP_SYNC
} command_type_sync_byte_t;

typedef enum 
{ 
    COMM_CMD_SET_CONN_DEV_LIST = 1,
    COMM_CMD_REQ_CONN_DEV_LIST,
    COMM_CMD_START,
    COMM_CMD_STOP,
    COMM_CMD_MEAS,
    COMM_CMD_SYNC,
    COMM_CMD_FREQUENCY,
    COMM_CMD_CALIBRATE,
    COMM_CMD_RESET,
    COMM_CMD_REQ_BATTERY_LEVEL,
    COMM_CMD_OK,
    COMM_CMD_TIME,
    COMM_CMD_CONN_DEV_UPDATE
} command_type_byte_t;

typedef enum 
{ 
    COMM_CMD_CONN_DEV_UPDATE_CONNECTED = 1,
    COMM_CMD_CONN_DEV_UPDATE_DISCONNECTED
} command_type_conn_dev_update_byte_t;


typedef enum
{
    COMM_CMD_CALIBRATION_START = 1,
    COMM_CMD_CALIBRATION_DONE,
    COMM_CMD_CALIBRATION_GYRO_DONE,
    COMM_CMD_CALIBRATION_ACCEL_DONE,
    COMM_CMD_CALIBRATION_MAG_DONE
} command_type_calibration_byte_t;

typedef enum
{
    COMM_CMD_SYNC_COMPLETE = 1,
    COMM_CMD_SYNC_LOST
} command_type_sync_type_byte_t;


typedef  struct
{
    int32_t w;
    int32_t x;
    int32_t y;
    int32_t z;
} stm32_quat_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} stm32_accel_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} stm32_gyro_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} stm32_compass_t;

typedef struct
{
    stm32_accel_t   accel;
    stm32_gyro_t    gyro;
    stm32_compass_t compass;
} stm32_raw_t;

#endif
