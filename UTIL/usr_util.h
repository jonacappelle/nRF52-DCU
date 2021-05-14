#ifndef _USR_UTIL_H__
#define _USR_UTIL_H__


#define PIN_CPU_ACTIVITY    19

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


void usr_gpio_init();

void check_cpu_activity();

uint32_t calculate_string_len(char * string);

#endif