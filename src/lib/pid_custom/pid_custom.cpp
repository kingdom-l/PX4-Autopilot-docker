#include "pid_custom.hpp"

void PIDCustom::abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

// 获取时间间隔
float PIDCustom::get_deltaT(hrt_abstime *then)
{
	hrt_abstime now = hrt_absolute_time();
	if(now < *then){
		return 0;
	}

	float dt = (float)(now - *then) / (float)1e6;

	*then = now;

	return dt;
}

// 梯形积分
void PIDCustom::f_trapezoid_intergral()
{
    // 计算梯形的面积,(上底+下底)*高/2
    _pid.ITerm = _pid.Ki * ((_pid.Err[1] + _pid.Err[0]) / 2) * _pid.dt;
}

/*
 * @brief  变速积分(误差大时积分作用更弱)
 * @note   当误差小时，正常积分；当误差大时，减小积分的作用。这样可以避免积分过度，导致系统的超调。
 */
void PIDCustom::f_changing_integration_rate()
{
    if (mx_abs(_pid.Err[0]) <= _pid.CoefB) // 如果误差小于等于阈值B，直接返回，对原先的积分不做更改
        return;
    if (mx_abs(_pid.Err[0]) <= (_pid.CoefA + _pid.CoefB)) // 如果误差介于B和A+B之间，乘以一个小于1的系数，减小积分作用
        _pid.ITerm *= (_pid.CoefA - mx_abs(_pid.Err[0]) + _pid.CoefB) / _pid.CoefA;
    else // 误差大于阈值A，直接忽略积分作用
    {
        _pid.ITerm = 0;
        _pid.Iout = 0;
    }
}

/*
 * @brief 微分先行(微分项只对get值进行微分，而不对设定值进行微分)
 * @note  在普通PID控制器中，微分项是基于误差的变化率。这意味着，如果设定值发生突变，微分项会产生一个突然的变化，这可能会导致系统的瞬时超调
 *        微分先行意味着，即使设定值发生突变，由于过程变量通常不能立即跟随设定值变化，微分项不会立即产生大的变化。
 *        因此，微分先行更适合于设定值频繁变化的场合。
 */
void PIDCustom::f_derivative_on_measurement()
{
    _pid.Dout = _pid.Kd * (_pid.get[1] - _pid.get[0]) / _pid.dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
void PIDCustom::f_derivative_filter()
{
    _pid.Dout = _pid.Dout * _pid.dt / (_pid.Derivative_LPF_RC + _pid.dt) +
                _pid.Last_Dout * _pid.Derivative_LPF_RC / (_pid.Derivative_LPF_RC + _pid.dt);
}

// 输出滤波
void PIDCustom::f_output_filter()
{
    _pid.Output = _pid.Output * _pid.dt / (_pid.Output_LPF_RC + _pid.dt) +
                  _pid.Last_Output * _pid.Output_LPF_RC / (_pid.Output_LPF_RC + _pid.dt);
}

float PIDCustom::forward_feed(Forward_Feed_s *instance, float in)
{
    if (instance->init_flag == 0)
    {
	get_deltaT(&(instance->DWT_CNT));
        instance->init_flag = 1;
        instance->xn_1 = in;
        return 0;
    }
    instance->dt = get_deltaT(&(instance->DWT_CNT));
    instance->xn = in;
    instance->Dout = (instance->fd) * (in - instance->xn_1) / instance->dt;
    instance->Kout = instance->fk * in;
    instance->Out = instance->Kout + instance->Dout;
    instance->xn_1 = in;

    return instance->Out;
}

/*
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDCustom::update_parameter(float kp, float ki, float maxout, float ilimit, float ea, float eb, float fk)
{
    _pid.Kp = kp;
    _pid.Ki = ki;
    _pid.MaxOut = maxout;
    _pid.IntegralLimit = ilimit;
    _pid.CoefA = ea;
    _pid.CoefB = eb;
    _pid.ff.fk = fk;
}



float PIDCustom::pid_calculate(float get, float set)
{
    if (_pid.init_flag == 0)
    {
	get_deltaT(&(_pid.DWT_CNT));
        _pid.init_flag = 1;
        return 0;
    }

    // 获取两次_pid计算的时间间隔,用于积分和微分
    _pid.dt = get_deltaT(&(_pid.DWT_CNT));
    // 保存上次的测量值和误差,计算当前error
    _pid.set[0] = set;
    _pid.get[0] = get;

    _pid.Err[0] = _pid.set[0] - _pid.get[0];


    // 如果在死区外,则计算PID
    if (mx_abs(_pid.Err[0]) > _pid.DeadBand)
    {
        // 基本的_pid计算,使用位置式
        _pid.Pout = _pid.Kp * _pid.Err[0];
        _pid.ITerm = _pid.Ki * _pid.Err[0] * _pid.dt;
        _pid.Dout = _pid.Kd * (_pid.Err[0] - _pid.Err[1]) / _pid.dt;

        // 梯形积分
        if (_pid.Improve & PID_Trapezoid_Intergral)
            f_trapezoid_intergral();
        // 变速积分
        if (_pid.Improve & PID_ChangingIntegrationRate)
            f_changing_integration_rate();
        // 微分先行
        if (_pid.Improve & PID_Derivative_On_Measurement)
            f_derivative_on_measurement();
        // 微分滤波器
        if (_pid.Improve & PID_DerivativeFilter)
            f_derivative_filter();

        _pid.Iout += _pid.ITerm;                         // 累加积分
        abs_limit(&(_pid.Iout), _pid.IntegralLimit);     // 积分限幅
        _pid.Output = _pid.Pout + _pid.Iout + _pid.Dout; // 计算输出
        if (_pid.Improve & PID_OutputFilter)             // 输出滤波
            f_output_filter();
        if (_pid.Improve & PID_FORWARD_FEEDBACK) // 前馈控制
            _pid.Output += forward_feed(&(_pid.ff), _pid.set[0]);
        abs_limit(&(_pid.Output), _pid.MaxOut); // 输出限幅
    }
    else // 进入死区, 则清空积分和输出
    {
        _pid.Output = 0;
        _pid.ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    _pid.get[1] = _pid.get[0];
    _pid.set[1] = _pid.set[0];
    _pid.Err[1] = _pid.Err[0];
    _pid.Last_Output = _pid.Output;
    _pid.Last_Dout = _pid.Dout;

    return _pid.Output;
}
