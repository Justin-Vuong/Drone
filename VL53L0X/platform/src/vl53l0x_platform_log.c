/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/*!
 * \file   VL53L0X_platform_log.c
 * \brief  Code function defintions for Ewok Platform Layer
 *
 */

#include <stdio.h>    // sprintf(), vsnprintf(), printf()
#include <stdarg.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "vl53l0x_def.h"
#include "vl53l0x_platform_log.h"

#include "UART.h"

#define trace_print(level, ...) trace_print_module_function(TRACE_MODULE_PLATFORM, level, TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) trace_print_module_function(TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#define    VL53L0X_MAX_STRING_LENGTH_PLT       256

char  debug_string[VL53L0X_MAX_STRING_LENGTH_PLT];


char * _trace_filename = NULL;
FILE *_tracefile = NULL;

uint32_t _trace_level = TRACE_LEVEL_WARNING;
uint32_t _trace_modules = TRACE_MODULE_NONE;
uint32_t _trace_functions = TRACE_FUNCTION_NONE;


void trace_print_module_function(uint32_t module, uint32_t level, uint32_t function, const char *format, ...)
{
    va_list arg_list;
    char message[VL53L0X_MAX_STRING_LENGTH_PLT];

    va_start(arg_list, format);
    vsnprintf(message, VL53L0X_MAX_STRING_LENGTH_PLT, format, arg_list);
    va_end(arg_list);

    USART3_SendString("VL53L0X: ");
    USART3_SendString(message);
    USART3_SendString("\n");
}

