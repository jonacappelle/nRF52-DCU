#ifndef _USR_INTERNAL_COMM_H__
#define _USR_INTERNAL_COMM_H__


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


void comm_parse_quat(uint8_t sensor_nr, int16_t w, int16_t x, int16_t y, int16_t z, uint8_t *data, uint32_t * len);


#endif