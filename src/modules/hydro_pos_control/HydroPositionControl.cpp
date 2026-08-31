/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "HydroPositionControl.hpp"

#include <px4_platform_common/events.h>

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector2d;
using matrix::Vector3f;
using matrix::wrap_pi;

HydroPositionControl::HydroPositionControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_hy_att_sp_pub(ORB_ID(hy_vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_dbg_val.value = 0.0f;
	_dbg_val.ind = 0;
	pub_dbg_val = orb_advertise(ORB_ID(debug_value), &_dbg_val);

	for(float &i :  _dbg_arr.data){
		i = 0.f;
	}
	_dbg_arr.id = 0;
	pub_dbg_arr = orb_advertise(ORB_ID(debug_array), &_dbg_arr);
}

HydroPositionControl::~HydroPositionControl()
{
	perf_free(_loop_perf);
}

bool
HydroPositionControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

float HydroPositionControl::saturate_function(float x, float max_value, float k, float center = 0)
{
	float out = (max_value - (-max_value)) / 2.f * tanhf(k * (x - center)) + (max_value + (-max_value)) / 2.f;
	return out;
}

float HydroPositionControl::mapForwardForceToThrottle(float force, float resolution, float force_scale) const
{
	const float safe_scale = math::max(fabsf(force_scale), 1e-3f);
	const float constrained_force = math::constrain(force, -safe_scale, safe_scale);
	const float constrained_resolution = math::constrain(resolution, 0.f, 1.f);

	if (constrained_force >= 0.f) {
		return constrained_resolution + constrained_force / safe_scale * (1.f - constrained_resolution);
	}

	return constrained_resolution + constrained_force / safe_scale * constrained_resolution;
}

float HydroPositionControl::mapPhysicalForwardForceToThrottle(float force, float resolution, float maximum_force) const
{
	if (!PX4_ISFINITE(force) || force <= 0.f) {
		return 0.f;
	}

	const float throttle = math::constrain(force / math::max(maximum_force, 1e-3f), 0.f, 1.f);
	return math::max(throttle, math::constrain(resolution, 0.f, 1.f));
}

void HydroPositionControl::resetControllerStates(int controller_mode, float depth_error, float depth_error_rate,
		float velocity_error)
{
	_depth_e_i = 0.f;
	_Va_e_i = 0.f;
	_depth_e_pre = depth_error;
	_Va_e_pre = velocity_error;
	_eadrc_hrp.reset(depth_error, depth_error_rate, velocity_error);
	_sact_plus.reset(velocity_error);
	_controller_mode_previous = controller_mode;
}


/**
 * @brief 计算加速度
 * @param[in] times就是下文中的N，必须为偶数，且不大于100次（100次就是100ms）
 *            velocity 速度
 * @return 是否收集到times次数据
 * @note  求取加速度有两个大问题：首先1000Hz下v和v_last可能是两个一样的、未刷新过的值
 *        再者，imu的速度本身就是有噪声的，就算两次速度不一样，但计算出的加速度有可能是噪声导致的。
 *        解决方案就是，用N次的速度来计算加速度，并且采取滑动采样实现1000Hz计算加速度
 */
uint8_t HydroPositionControl::TimeDerivativeCalc(uint8_t times, time_derivative_t *ins, float position)
{
	if (ins->init_flag == 0)
	{
		ins->last_time = hrt_absolute_time();
		ins->init_flag = 1;
		ins->pos[ins->index++] = position;
		return 0;
	}
	_time_now = hrt_absolute_time();
	ins->dt[(ins->index - 1) % times] = (_time_now - ins->last_time) * 1e-6f;
	ins->last_time = _time_now;
	ins->pos[ins->index % times] = position;
	ins->temp_time_sum += ins->dt[(ins->index - 1) % times];
	if (ins->index >= times / 2) // 已经超过一半的次数了，可以开始计算了
	{
		ins->temp_res_sub[ins->index % (times / 2)] = ((ins->pos[ins->index % times] - ins->pos[(ins->index - times / 2) % times]) / ins->temp_time_sum);
		ins->temp_res += ins->temp_res_sub[ins->index % (times / 2)];
		ins->temp_time_sum -= ins->dt[(ins->index - times / 2) % times];
		if ((ins->index + 1) >= times) // 如果已经达到了给定的次数，可以计算加速度
		{
		ins->vel = ins->temp_res / ((times / 2) * 1.0f);
		ins->temp_res -= ins->temp_res_sub[(ins->index + 1) % (times / 2)];
		ins->index++;
		return 1;
		}
	}
	ins->index++;
	return 0;
}

/*
 * @brief 积分分离
 * @param e 当前误差
 *        e_pre 上一次误差
 *        dt 时间间隔
 *        a,b 积分分离阈值
 * @retval 积分项输出
 */
// float HydroPositionControl::anti_windup(float e, float e_pre, float dt, float a, float b, )
// {
// 	static float iout = 0.f; // 积分项输出
// 	if(std::fabs(e) <= a){
// 		iout = (e_pre + e) * 0.5f * dt + iout;
// 	}
// 	else if(std::fabs(e) <= (a + b))
// 	{
// 		iout = _Va_e_i + (e_pre + e) * 0.5f * dt * (b - std::fabs(e) + a) / b;
// 	}
// 	else
// 	{
// 		iout = 0.f;
// 	}
// 	return iout;
// }

void
HydroPositionControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// ****** 注释掉，避免TD输出nan ******
	// if (_local_pos_sub.update(&_local_pos)) {
	// ****** 注释掉，避免TD输出nan ******
	if (_debug_vect_sub.update(&_debug_vec)) {

		 if (_parameter_update_sub.updated()) {
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);

			// 如果有任何参数更新, 调用 updateParams() 来检查
			// 该类属性是否需要更新 (然后执行更新)。
			updateParams();
		}

		float dt = 0.f;

		static constexpr float DT_MIN = 0.01f;
		static constexpr float DT_MAX = 0.05f;

		vehicle_attitude_s att{};

		if (_att_sub.copy(&att)) {
			dt = math::constrain((att.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = att.timestamp_sample;

			// get current rotation matrix and euler angles from control state quaternions
			_R = matrix::Quatf(att.q);
		}
		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		// ****** 测试低通滤波器 ******
		// 对位置进行低通滤波
		// _dbg_key.timestamp = hrt_absolute_time(); // or
		// _dbg_val.timestamp = hrt_absolute_time();
		// _dbg_val.value = _pos_x_lpf.apply(debug_val.x);
		// orb_publish(ORB_ID(debug_value), pub_dbg_val, &_dbg_val);
		// ****** 测试低通滤波器 ******

		// ****** 测试位置对时间求导 ******
		const bool vx_ready = TimeDerivativeCalc(25, &_posx_derivate, (double)_debug_vec.x);
		const bool vy_ready = TimeDerivativeCalc(25, &_posy_derivate, (double)_debug_vec.y);
		TimeDerivativeCalc(25, &_posz_derivate, (double)_debug_vec.z);
		const bool derivative_ready = vx_ready && vy_ready;
		// printf("ad %p %p %p\n", &_posx_derivate, &_posy_derivate, &_posz_derivate);
		// ****** 测试位置对时间求导 ******

		// ****** 测试TD ******
		// 使用TD估计速度，并发布debug_value消息，在mavlink inspector显示
		// ****** 2024-1231注释 ****** // 20250527问题：在位置为0时，速度估计有值()
		// _pos_x_td.set_params(_param_hy_pos_td_h.get(), _param_hy_pos_td_r0.get(), _param_hy_pos_td_h0.get());
		// _pos_x_td.update(_debug_vec.x);
		// _vx_hat = _pos_x_td.getDerivative();
		// _px_hat = _pos_x_td.getSmoothedSignal();
		// printf("pos_x: %f %f %f\n", (double)_debug_vec.x, (double)_px_hat, (double)_vx_hat);

		// _pos_y_td.set_params(_param_hy_pos_td_h.get(), _param_hy_pos_td_r0.get(), _param_hy_pos_td_h0.get());
		// _pos_y_td.update(_debug_vec.y);
		// _vy_hat = _pos_y_td.getDerivative();
		// _py_hat = _pos_y_td.getSmoothedSignal();
		// printf("pos_y: %f %f %f\n", (double)_debug_vec.y, (double)_py_hat, (double)_vy_hat);

		// _pos_z_td.set_params(_param_hy_pos_td_h.get(), _param_hy_pos_td_r0.get(), _param_hy_pos_td_h0.get());
		// _pos_z_td.update(_debug_vec.z);
		// _vz_hat = _pos_z_td.getDerivative();
		// _pz_hat = _pos_z_td.getSmoothedSignal();
		// printf("pos_z: %f %f %f\n", (double)_debug_vec.z, (double)_pz_hat, (double)_vz_hat);


		// _dbg_val.value = _px_hat; // 判断一下TD的滤波输出如何
		// _dbg_val.timestamp = hrt_absolute_time();
		// orb_publish(ORB_ID(debug_value), pub_dbg_val, &_dbg_val);

		// ****** debug_array无法分开显示data数据内容(舍弃) ******
		// _dbg_arr.timestamp = hrt_absolute_time();
		// _dbg_arr.data[0] = debug_vec.x;
		// _dbg_arr.data[1] = _pos_x_td.getSmoothedSignal(); // 判断一下TD的滤波输出如何
		// _dbg_arr.data[2]= _pos_x_td.getDerivative(); // 判断一下TD的速度估计如何
		// orb_publish(ORB_ID(debug_array), pub_dbg_arr, &_dbg_arr);
		// ****** debug_array无法分开显示data数据内容(舍弃) ******

		// ****** 测试TD ******

		/************ 获得速度和深度信息 ************/
		//_Va_hat = sqrtf(_vx_hat * _vx_hat + _vy_hat * _vy_hat + _vz_hat * _vz_hat); // TD估计的速度
		// _Va_hat = sqrtf(_vx_hat * _vx_hat + _vy_hat * _vy_hat);
		_Va_hat = sqrtf(_posx_derivate.vel * _posx_derivate.vel + _posy_derivate.vel * _posy_derivate.vel);
		float Va_sp = _param_hy_va_sp.get();

		// ****** 订阅动捕测量的深度信息，仅单个维度(高度) ******
		// struct debug_key_value_s debug_value;
		// float depth = -debug_value.value; //_local_pos.z; // 向下为正
		// ****** 订阅动捕测量的深度信息，仅单个维度(高度) ******

		// ****** 2024-1231注释 ******
		// struct debug_vect_s debug_vec; // 订阅动捕测量的位置信息
		// _debug_vect_sub.copy(&_debug_vec);
		float depth = _debug_vec.z; // depth遵循水平面以上为正，水平面以下为负
		// ****** 2024-1231注释 ******

		// ****** 订阅深度计的深度信息 ******
		// _depth_estimated_sub.update(&_depth_estimated);
		// float depth = -_depth_estimated.depth_estimated;
		// printf("depth_estimated: %f \n", (double)depth);
		// ****** 订阅深度计的深度信息 ******

		float depth_sp = _param_hy_depth_sp.get(); // 遵循海平面以上为正，海平面以下为负
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);
		_vehicle_status_sub.copy(&_vehicle_status);

		/************ 获得速度和深度信息 ************/
		// _Va_hat = 0; // 用于调试ESO是否饱和
		_Va_e = Va_sp - _Va_hat;
		_depth_e = depth_sp - depth;
		const float depth_error_rate = -_posz_derivate.vel;
		// const int controller_mode = math::constrain(_param_hy_depva_pid_en.get(), ControllerAdrc, ControllerSactPlus);
		const int controller_mode = math::constrain<int32_t>(_param_hy_depva_pid_en.get(), ControllerAdrc, ControllerSactPlus);

		if (controller_mode != _controller_mode_previous) {
			resetControllerStates(controller_mode, _depth_e, depth_error_rate, _Va_e);
			_eadrc_active = false;
			_sact_active = false;
		}

		if (controller_mode == ControllerPid) {
			/************ 速度PID控制 ************/
			const float ve_a = _param_hy_ve_a.get();
			const float ve_b = math::max(_param_hy_ve_b.get(), 1e-4f);

			if (fabsf(_Va_e) <= ve_a) {
				_Va_e_i += _param_hy_va_i.get() * (_Va_e_pre + _Va_e) * 0.5f * dt;

			} else if (fabsf(_Va_e) <= ve_a + ve_b) {
				_Va_e_i += _param_hy_va_i.get() * (_Va_e_pre + _Va_e) * 0.5f * dt
					   * (ve_b - fabsf(_Va_e) + ve_a) / ve_b;

			} else {
				_Va_e_i = 0.f;
			}

			_Va_e_pre = _Va_e;
			_Va_e_i = math::constrain(_Va_e_i, -_param_hy_ve_ilimit.get(), _param_hy_ve_ilimit.get());
			const float fx_force = _param_hy_va_p.get() * _Va_e + _Va_e_i + _param_hy_va_ff.get() * Va_sp;
			_fx_sp = mapForwardForceToThrottle(fx_force, _param_hy_ve_res.get(), _param_hy_vfx_sp_slope.get());

			/************ 深度PID控制 ************/
			const float vel_fb = _param_hy_velfb_p.get() * _Va_e;
			const float de_a = _param_hy_de_a.get();
			const float de_b = math::max(_param_hy_de_b.get(), 1e-4f);

			if (fabsf(_depth_e) <= de_a) {
				_depth_e_i += _param_hy_dep_i.get() * (_depth_e_pre + _depth_e) * 0.5f * dt;

			} else if (fabsf(_depth_e) <= de_a + de_b) {
				_depth_e_i += _param_hy_dep_i.get() * (_depth_e_pre + _depth_e) * 0.5f * dt
					      * (de_b - fabsf(_depth_e) + de_a) / de_b;

			} else {
				_depth_e_i = 0.f;
			}

			_depth_e_pre = _depth_e;
			_depth_e_i = math::constrain(_depth_e_i, -_param_hy_de_ilimit.get(), _param_hy_de_ilimit.get());
			_fz_sp = math::constrain(_param_hy_dep_p.get() * _depth_e + vel_fb + _depth_e_i
						 - _param_hy_dep_ff.get(), -_param_hy_dep_lim.get(), _param_hy_dep_lim.get());

			// Keep the legacy observers alive for PID diagnostics.
			_vel_eso.set_params(1.f / _param_hy_v_eso_b0_inv.get(), _param_hy_v_eso_beta1.get(),
					    _param_hy_v_eso_beta2.get(), _param_hy_v_eso_h.get());
			_vel_eso.update(_fx_sp, _Va_hat);
			_depth_eso.set_params(1.f / _param_hy_d_eso_b0_inv.get(), _param_hy_d_eso_beta1.get(),
					      _param_hy_d_eso_beta2.get(), _param_hy_d_eso_beta3.get(), _param_hy_d_eso_h.get());
			_depth_eso.update(_fz_sp, depth);

		} else if (controller_mode == ControllerAdrc) {
			/************ 原PX4 ADRC控制 ************/
			const float vel_b0_inv = math::max(_param_hy_v_eso_b0_inv.get(), 1e-3f);
			const float dep_b0_inv = math::max(_param_hy_d_eso_b0_inv.get(), 1e-3f);
			_vel_eso.set_params(1.f / vel_b0_inv, _param_hy_v_eso_beta1.get(), _param_hy_v_eso_beta2.get(),
					    _param_hy_v_eso_h.get());
			_vel_eso.update(_fx_sp, _Va_hat);
			_depth_eso.set_params(1.f / dep_b0_inv, _param_hy_d_eso_beta1.get(), _param_hy_d_eso_beta2.get(),
					      _param_hy_d_eso_beta3.get(), _param_hy_d_eso_h.get());
			_depth_eso.update(_fz_sp, depth);

			const float fx_force = (_param_hy_va_adrc_p.get() * _Va_e - _vel_eso.getTotalDisturbance()
						+ _param_hy_va_ff_adrc.get() * Va_sp) * vel_b0_inv;
			_fx_sp = mapForwardForceToThrottle(fx_force, _param_hy_ve_res_adrc.get(),
						 _param_hy_vfx_sp_slpadrc.get());
			_fz_sp = math::constrain(_param_hy_dep_adrc_p.get() * _depth_e
						 - _param_hy_dep_adrc_d.get() * _depth_eso.getStateDotEst()
						 - _depth_eso.getTotalDisturbance() * _param_hy_dep_kcomp_eso.get() * dep_b0_inv
						 - _param_hy_dep_ff_adrc.get(), -_param_hy_dep_lim_adrc.get(),
						 _param_hy_dep_lim_adrc.get());

		} else if (controller_mode == ControllerEadrcHrp) {
			/************ 轻量eADRC-HRP控制 ************/
			const float maximum_forward_force = 2.f * math::max(_param_hy_thrust_max.get(), 1e-3f);
			EadrcHrpParams params{};
			params.depth_b0_inverse = _param_hy_hr_d_b0_inv.get();
			params.velocity_b0_inverse = _param_hy_hr_v_b0_inv.get();
			params.depth_kp = _param_hy_hr_d_kp.get();
			params.depth_kd = _param_hy_hr_d_kd.get();
			params.depth_observer_bandwidth = _param_hy_hr_d_wo.get();
			params.velocity_kp = _param_hy_hr_v_kp.get();
			params.velocity_observer_bandwidth = _param_hy_hr_v_wo.get();
			params.depth_alpha = _param_hy_hr_d_alp.get();
			params.velocity_alpha = _param_hy_hr_v_alp.get();
			params.residual_lpf = _param_hy_hr_rlpf.get();
			params.confidence_time_constant = _param_hy_hr_c_tc.get();
			params.compensation_ramp_time = _param_hy_hr_ramp.get();
			params.depth_feedforward = _param_hy_hr_dep_ff.get();
			params.velocity_feedforward = _param_hy_hr_va_ff.get() * Va_sp;
			params.depth_force_limit = _param_hy_dep_lim_adrc.get();
			params.velocity_force_limit = maximum_forward_force;
			params.depth_predictor.feature_count = HrpPredictor::DepthFeatureCount;
			params.depth_predictor.forgetting_factor = _param_hy_hr_forget.get();
			params.depth_predictor.ridge = _param_hy_hr_ridge.get();
			params.depth_predictor.prediction_limit = _param_hy_hr_d_rlim.get();
			params.depth_predictor.noise_sigma = _param_hy_hr_d_sig.get();
			params.velocity_predictor = params.depth_predictor;
			params.velocity_predictor.feature_count = HrpPredictor::VelocityFeatureCount;
			params.velocity_predictor.prediction_limit = _param_hy_hr_v_rlim.get();
			params.velocity_predictor.noise_sigma = _param_hy_hr_v_sig.get();

			// The basic Kp/Kd controller is always evaluated so its unsaturated
			// output can be inspected while the vehicle is stationary.
			const float fz_base_raw = params.depth_b0_inverse
						  * (params.depth_kp * _depth_e + params.depth_kd * depth_error_rate)
						  - params.depth_feedforward;
			const float fx_base_raw = params.velocity_b0_inverse * params.velocity_kp * _Va_e
						  + params.velocity_feedforward;
			const float fz_utilization = fabsf(fz_base_raw)
						     / math::max(fabsf(params.depth_force_limit), 1e-3f);
			const float fx_utilization = fabsf(fx_base_raw)
						     / math::max(fabsf(params.velocity_force_limit), 1e-3f);

			const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
			const bool emergency_throttle_cut = PX4_ISFINITE(_manual_control_setpoint.throttle)
							    && fabsf(_manual_control_setpoint.throttle + 1.f) < 0.1f;
			const bool eadrc_enable = armed && derivative_ready && !emergency_throttle_cut;

			if (!eadrc_enable) {
				// Do not update the ESO or HRP predictor without actuator authority.
				// The raw Kp/Kd output remains available only as a diagnostic.
				if (_eadrc_active) {
					_eadrc_hrp.reset(_depth_e, depth_error_rate, _Va_e);
				}

				_eadrc_active = false;
				_fx_sp = 0.f;
				_fz_sp = 0.f;
				_eadrc_hrp.setAppliedForces(0.f, 0.f);

				_dbg_arr.timestamp = hrt_absolute_time();
				_dbg_arr.id = 20;
				_dbg_arr.data[0] = fx_base_raw;
				_dbg_arr.data[1] = fz_base_raw;
				_dbg_arr.data[2] = fx_utilization;
				_dbg_arr.data[3] = fz_utilization;
				_dbg_arr.data[4] = fx_utilization >= 1.f ? 1.f : 0.f;
				_dbg_arr.data[5] = fz_utilization >= 1.f ? 1.f : 0.f;
				orb_publish(ORB_ID(debug_array), pub_dbg_arr, &_dbg_arr);

			} else {
				if (!_eadrc_active) {
					// Align the observer with the current tracking error and clear all
					// HRP samples before applying control.
					_eadrc_hrp.reset(_depth_e, depth_error_rate, _Va_e);
					_eadrc_active = true;
				}

				const EadrcHrpController::Output output =
					_eadrc_hrp.update(dt, _depth_e, depth_error_rate, _Va_e, params);
				_fz_sp = output.fz_force;
				_fx_sp = math::constrain(output.fx_force / maximum_forward_force, 0.f, 1.f);
				_eadrc_hrp.setAppliedForces(_fx_sp * maximum_forward_force, _fz_sp);
			}

		} else {
			/************ 固定startup参数SACT+控制 ************/
			const float maximum_forward_force = 2.f * math::max(_param_hy_thrust_max.get(), 1e-3f);
			SactPlusParams params{};
			params.depth_b0_inverse = _param_hy_sa_d_b0_inv.get();
			params.velocity_b0_inverse = _param_hy_sa_v_b0_inv.get();
			params.derivative_filter_time_constant = _param_hy_sa_der_tc.get();
			params.compensation_ramp_time = _param_hy_sa_ramp.get();
			params.depth_feedforward = _param_hy_sa_dep_ff.get();
			params.velocity_feedforward = _param_hy_sa_va_ff.get() * Va_sp;
			params.depth_force_limit = _param_hy_dep_lim_adrc.get();
			params.velocity_force_limit = maximum_forward_force;
			params.depth.proportional_scale = _param_hy_sa_d_sp.get();
			params.depth.derivative_scale = _param_hy_sa_d_sd.get();
			params.depth.proportional_boundary = _param_hy_sa_d_dp.get();
			params.depth.derivative_boundary = _param_hy_sa_d_dd.get();
			params.depth.proportional_exponent = _param_hy_sa_d_dlp.get();
			params.depth.derivative_exponent = _param_hy_sa_d_dld.get();
			params.depth.alpha1 = _param_hy_sa_d_a1.get();
			params.depth.alpha2 = _param_hy_sa_d_a2.get();
			params.depth.gamma = _param_hy_sa_d_gam.get();
			params.depth.nominal_disturbance = _param_hy_sa_d_dnm.get();
			params.depth.theta_limit = _param_hy_sa_d_tlm.get();
			params.depth.lambda_limit = _param_hy_sa_d_llm.get();
			params.velocity.proportional_scale = _param_hy_sa_v_sp.get();
			params.velocity.derivative_scale = _param_hy_sa_v_sd.get();
			params.velocity.proportional_boundary = _param_hy_sa_v_dp.get();
			params.velocity.derivative_boundary = _param_hy_sa_v_dd.get();
			params.velocity.proportional_exponent = _param_hy_sa_v_dlp.get();
			params.velocity.derivative_exponent = _param_hy_sa_v_dld.get();
			params.velocity.alpha1 = _param_hy_sa_v_a1.get();
			params.velocity.alpha2 = _param_hy_sa_v_a2.get();
			params.velocity.gamma = _param_hy_sa_v_gam.get();
			params.velocity.nominal_disturbance = _param_hy_sa_v_dnm.get();
			params.velocity.theta_limit = _param_hy_sa_v_tlm.get();
			params.velocity.lambda_limit = _param_hy_sa_v_llm.get();

			const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
			const bool emergency_throttle_cut = PX4_ISFINITE(_manual_control_setpoint.throttle)
							    && fabsf(_manual_control_setpoint.throttle + 1.f) < 0.1f;
			const bool sact_enable = armed && derivative_ready && !emergency_throttle_cut;

			if (!sact_enable) {
				// Continue evaluating nonlinear PD and fixed feedforward for static
				// diagnostics, but clear/freeze Lambda and Theta and command no force.
				_sact_active = false;
				const SactPlusController::Output base_output =
					_sact_plus.update(dt, _depth_e, depth_error_rate, _Va_e, params, false);
				_fx_sp = 0.f;
				_fz_sp = 0.f;

				const float fx_utilization = fabsf(base_output.fx_base_raw)
							     / math::max(fabsf(params.velocity_force_limit), 1e-3f);
				const float fz_utilization = fabsf(base_output.fz_base_raw)
							     / math::max(fabsf(params.depth_force_limit), 1e-3f);
				_dbg_arr.timestamp = hrt_absolute_time();
				_dbg_arr.id = 21;
				_dbg_arr.data[0] = base_output.fx_base_raw;
				_dbg_arr.data[1] = base_output.fz_base_raw;
				_dbg_arr.data[2] = fx_utilization;
				_dbg_arr.data[3] = fz_utilization;
				_dbg_arr.data[4] = fx_utilization >= 1.f ? 1.f : 0.f;
				_dbg_arr.data[5] = fz_utilization >= 1.f ? 1.f : 0.f;
				orb_publish(ORB_ID(debug_array), pub_dbg_arr, &_dbg_arr);

			} else {
				if (!_sact_active) {
					// Clear both adaptive channels on the enable edge while leaving
					// the continuously evaluated nonlinear PD path intact.
					_sact_plus.resetAdaptiveStates();
					_sact_active = true;
				}

				const SactPlusController::Output output =
					_sact_plus.update(dt, _depth_e, depth_error_rate, _Va_e, params, true);
				_fz_sp = output.fz_force;
				_fx_sp = mapPhysicalForwardForceToThrottle(output.fx_force, _param_hy_ve_res_adrc.get(),
								 maximum_forward_force);
			}
		}

		// Unified diagnostics. The output interface remains unchanged:
		// thrust_body[0] is normalized throttle and thrust_body[2] is force in N.
		_pos_sp.timestamp = hrt_absolute_time();
		_pos_sp.x = _Va_hat;
		_pos_sp.y = _fx_sp;
		_pos_sp.z = depth;
		_pos_sp.vx = _fz_sp;
		_pos_sp.yawspeed = static_cast<float>(controller_mode);

		if (controller_mode == ControllerEadrcHrp) {
			_pos_sp.vy = _eadrc_hrp.velocityState();
			_pos_sp.vz = _eadrc_hrp.velocityDisturbance();
			_pos_sp.acceleration[0] = _eadrc_hrp.depthState();
			_pos_sp.acceleration[1] = _eadrc_hrp.depthRateState();
			_pos_sp.acceleration[2] = _eadrc_hrp.depthDisturbance();
			_pos_sp.yaw = _eadrc_hrp.depthPrediction();

		} else if (controller_mode == ControllerSactPlus) {
			_pos_sp.vy = _sact_plus.velocityLambda();
			_pos_sp.vz = _sact_plus.velocityTheta();
			_pos_sp.acceleration[0] = _sact_plus.depthLambda();
			_pos_sp.acceleration[1] = _sact_plus.depthTheta();
			_pos_sp.acceleration[2] = _Va_e;
			_pos_sp.yaw = _depth_e;

		} else if (controller_mode == ControllerAdrc) {
			_pos_sp.vy = _vel_eso.getStateEst();
			_pos_sp.vz = _vel_eso.getTotalDisturbance();
			_pos_sp.acceleration[0] = _depth_eso.getStateEst();
			_pos_sp.acceleration[1] = _depth_eso.getStateDotEst();
			_pos_sp.acceleration[2] = _depth_eso.getTotalDisturbance();
			_pos_sp.yaw = _depth_e;
		}else{
			_pos_sp.vy = _Va_e;
			_pos_sp.vz = _Va_e_i;
			_pos_sp.acceleration[0] = _depth_e;
			_pos_sp.acceleration[1] = _depth_e_i;
		}

		_vehicle_local_pos_sp_pub.publish(_pos_sp);

		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		const matrix::Eulerf euler_angles(_R);
		vehicle_attitude_setpoint_s att_sp{};
		att_sp.timestamp = hrt_absolute_time();
		att_sp.roll_body = _manual_control_setpoint.roll * radians(_param_hy_d_rmax.get()); // rad roll的手动控制反应很慢
		att_sp.pitch_body = 0; //-_manual_control_setpoint.pitch * radians(_param_hy_d_pmax.get());// rad
		att_sp.yaw_body = euler_angles.psi();
		att_sp.thrust_body[0] = _fx_sp; // 最大油门量为1
		att_sp.thrust_body[2] = _fz_sp;

		// ****** 显示TD估计结果 ******
		// _pos_sp.timestamp = hrt_absolute_time();
		// _pos_sp.x = _debug_vec.x;
		// _pos_sp.y = _debug_vec.y;
		// _pos_sp.z = _debug_vec.z;
		// _pos_sp.vx = _px_hat;
		// _pos_sp.vy = _vx_hat;
		// _pos_sp.vz = _Va_hat;
		// _pos_sp.acceleration[0] = _posx_derivate.vel;
		// // _pos_sp.acceleration[1] = _vy_hat; // 判断一下TD的滤波输出如何
		// _pos_sp.acceleration[1] = _posy_derivate.vel; // 判断一下TD的滤波输出如何
		// _pos_sp.acceleration[2] = _posz_derivate.vel; // 判断一下TD的速度估计如何
		// _vehicle_local_pos_sp_pub.publish(_pos_sp);
		// ****** 显示TD估计结果 ******

		_hy_att_sp_pub.publish(att_sp);

		// printf("h vel sp:%f %f e:%f e_i:%f fx_sp:%f\n", (double)Va_sp, (double)_Va_hat, (double)_Va_e, (double)_Va_e_i, (double)fx_sp);
		// printf("h dep sp:%f %f e:%f e_i:%f fz_sp:%f\n", (double)depth_sp, (double)depth, (double)_depth_e, (double)_depth_e_i, (double)fz_sp); // (double)pitch_sp_sat);
		// printf("h att r_sp:%f p_sp:%f \n", (double)att_sp.roll_body, (double)att_sp.pitch_body);// (double)pitch_sp_sat);
		// printf("h thrust_sp: %f %f \n", (double)att_sp.thrust_body[0], (double)att_sp.thrust_body[2]);
	}

	perf_end(_loop_perf);
}


int HydroPositionControl::task_spawn(int argc, char *argv[])
{
	HydroPositionControl *instance = new HydroPositionControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int HydroPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HydroPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_pos_control is the fixed-wing position controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hydro_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	//PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int hydro_pos_control_main(int argc, char *argv[])
{
	return HydroPositionControl::main(argc, argv);
}
