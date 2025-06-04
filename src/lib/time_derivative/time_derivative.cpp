/**
 * @file time_derivative.cpp
 */

#include "time_derivative.hpp"

static hrt_abstime last_run = 0, time_now_us = 0;

/**
 * @brief 计算加速度
 * @param[in] times就是下文中的N，必须为偶数，且不大于100次（100次就是100ms）
 *            velocity 速度
 * @return 是否收集到times次数据
 * @note  求取加速度有两个大问题：首先1000Hz下v和v_last可能是两个一样的、未刷新过的值
 *        再者，imu的速度本身就是有噪声的，就算两次速度不一样，但计算出的加速度有可能是噪声导致的。
 *        解决方案就是，用N次的速度来计算加速度，并且采取滑动采样实现1000Hz计算加速度
 */
uint8_t TimeDerivativeCalc(uint8_t times, time_derivative_t *ins, double position)
{
    static uint32_t index = 0;
    static float temp_res = 0, temp_time_sum = 0, temp_res_sub[50] = {0}; // 便于计算的中间量
    if (ins->init_flag == 0)
    {
	last_run = hrt_absolute_time();
        ins->init_flag = 1;
        ins->pos[index++] = position;
        return 0;
    }
    time_now_us = hrt_absolute_time();
    ins->dt[(index - 1) % times] = (time_now_us - last_run) * 1e-6f;
    last_run = time_now_us;
    ins->pos[index % times] = position;
    temp_time_sum += ins->dt[(index - 1) % times];
    if (index >= times / 2) // 已经超过一半的次数了，可以开始计算了
    {
        temp_res_sub[index % (times / 2)] = ((ins->pos[index % times] - ins->pos[(index - times / 2) % times]) / temp_time_sum);
        temp_res += temp_res_sub[index % (times / 2)];
        temp_time_sum -= ins->dt[(index - times / 2) % times];
        if ((index + 1) >= times) // 如果已经达到了给定的次数，可以计算加速度
        {
            ins->vel = temp_res / ((times / 2) * 1.0f);
            temp_res -= temp_res_sub[(index + 1) % (times / 2)];
            index++;
            return 1;
        }
    }
    index++;
    return 0;
}
