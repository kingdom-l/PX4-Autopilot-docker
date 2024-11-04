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
	_attitude_sp_pub(ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{

}

HydroPositionControl::~HydroPositionControl()
{
	perf_free(_loop_perf);
}

bool
HydroPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
HydroPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_local_pos_sub.update(&_local_pos)) {

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

		// 订阅动捕测量的深度信息
		struct debug_key_value_s debug_value;
		_debug_sub.copy(&debug_value);
		float depth = -debug_value.value; //_local_pos.z; // 向下为正

		// 订阅深度计的深度信息
		// _depth_estimated_sub.update(&_depth_estimated);
		// float depth = _depth_estimated.depth_estimated;

		float depth_sp = -_param_hy_depth_sp.get();

		float depth_e = depth_sp - depth;
		_depth_e_i = _depth_e_i + dt / 2 * (depth_e + _depth_e_pre);
		_depth_e_pre = depth_e;

		float pitch_sp_sat = math::constrain(_param_hy_high_p.get()*depth_e+_param_hy_high_i.get()*_depth_e_i, -radians(_param_hy_p_lim.get()), radians(_param_hy_p_lim.get()));

		if(std::fabs(_param_hy_high_i.get()) > 1e-6f){

			float pitch_sp_unsat = _param_hy_high_p.get()*depth_e+_param_hy_high_i.get()*_depth_e_i;
			_depth_e_i = _depth_e_i + dt/(radians(_param_hy_high_i.get())) * (pitch_sp_sat - pitch_sp_unsat);

		}

		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		const matrix::Eulerf euler_angles(_R);
		vehicle_attitude_setpoint_s att_sp{};
		att_sp.timestamp = hrt_absolute_time();
		att_sp.roll_body = _manual_control_setpoint.roll * radians(_param_hy_r_lim.get()); // roll的手动控制反应很慢
		att_sp.pitch_body = -pitch_sp_sat; // rad
		att_sp.yaw_body = euler_angles.psi();
		att_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f; // 最大油门量为0.7
		_attitude_sp_pub.publish(att_sp);

		// printf("pos: %f %f %f %f %f\n", (double)depth_sp, (double)depth, (double)depth_e, (double)_depth_e_i, (double)pitch_sp_sat);
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
