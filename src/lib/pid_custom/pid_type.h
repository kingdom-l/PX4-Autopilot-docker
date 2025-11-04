/**
 * @file pid_type.h
 *
 */
#ifndef PID_TYPE_H
#define PID_TYPE_H

#include "stdint.h"
#include "memory.h"
#include <drivers/drv_hrt.h>

#ifndef mx_abs
#define mx_abs(x) ((x > 0) ? x : -x)
#endif

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
enum PID_Improvement_e : uint8_t
{
    PID_IMPROVE_NONE = 0,              // 0000 0000
    PID_Derivative_On_Measurement = 1 << 0, // 微分先行
    PID_Trapezoid_Intergral = 1 << 2,       // 梯形积分
    PID_OutputFilter = 1 << 3,              // 输出滤波
    PID_ChangingIntegrationRate = 1 << 4,   // 变速积分
    PID_DerivativeFilter = 1 << 5,          // 微分滤波，一阶导会带来高频噪声
    PID_FORWARD_FEEDBACK = 1 << 6,          // 使用前馈控制

};

typedef struct
{
    float xn;          // 输入
    float xn_1;        // 上次输入
    float fd;          // 前馈微分系数
    float fk;          // 前馈比例系数
    float Kout;        // 比例输出
    float Dout;        // 微分输出
    float Out;         // 输出
    uint8_t init_flag; // 标记是否初始化
    hrt_abstime DWT_CNT;  // 上次前馈计算时的DWT计数值，用于计算两次前馈计算的时间间隔
    float dt;          // 两次前馈计算的时间间隔

} Forward_Feed_s;

/* PID结构体 */
typedef struct
{
    // 基础参数
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // 用于提升PID性能的参数
    PID_Improvement_e Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系

    // 用于PID计算的参数
    float get[2]; // 0为当前get，1为上一次get
    float Err[2]; // 0为当前误差，1为上一次误差
    float set[2]; // 目标值

    // PID计算的结果
    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float Output;
    float Last_Output;
    float Last_Dout;

    // 前馈控制
    Forward_Feed_s ff; // forward feedback

    // 用于计算两个PID计算的时间间隔
    uint8_t init_flag; // 标记是否初始化
    hrt_abstime DWT_CNT;  // 上次PID计算时的DWT计数值，用于计算两次PID计算的时间间隔
    float dt;          // 两次PID计算的时间间隔

} PIDInstance;


#endif // !PID_H
