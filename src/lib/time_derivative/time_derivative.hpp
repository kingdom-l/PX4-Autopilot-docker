/**
 * @file time_derivative.hpp
 */
#pragma once

#include <drivers/drv_hrt.h>

typedef struct
{
    double vel;           // 速度
    double pos[100];      // 当前位置

    // 用于计算dt，从而计算速度
    uint8_t init_flag; // 标记是否初始化
    double dt[100];
} time_derivative_t;

uint8_t TimeDerivativeCalc(uint8_t times, time_derivative_t *ins, double position);
