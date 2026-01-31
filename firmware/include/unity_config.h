/**
 * @file unity_config.h
 * @brief Unity Test Framework Configuration
 */

#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#include <stdio.h>
#include <stdint.h>

// Use standard putchar for output (works with RP2040 USB CDC)
#define UNITY_OUTPUT_CHAR(c) putchar(c)
#define UNITY_OUTPUT_FLUSH() fflush(stdout)

// Integer types
#define UNITY_INT_WIDTH 32
#define UNITY_POINTER_WIDTH 32

#endif // UNITY_CONFIG_H
