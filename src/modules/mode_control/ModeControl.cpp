/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "ModeControl.hpp"

#include <include/HyModeName.hpp>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

#include <matrix/math.hpp>

int ModeControl::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int ModeControl::custom_command(int argc, char *argv[])
{

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}
/*
	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int ModeControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mode_control",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ModeControl *ModeControl::instantiate(int argc, char *argv[])
{
	/*int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}*/

	ModeControl *instance = new ModeControl();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

ModeControl::ModeControl()
	: ModuleParams(nullptr)
{
}

void ModeControl::run()
{
	PX4_INFO("ModeControl start");

	// initialize parameters
	parameters_update(true);

	usleep(1000000);

	_last_register_time = hrt_absolute_time();

	while (!should_exit()) {

		usleep(1_ms);

		if (_registered_mode_num < 8 && hrt_absolute_time() - _last_register_time > 50_ms)
		{
			_reg_request_msg.request_id = _registered_mode_num + 1;
			_reg_request_msg.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
			_reg_request_msg.register_arming_check = true;
			_reg_request_msg.register_mode = true;
			_reg_request_msg.register_mode_executor = false;
			_reg_request_msg.enable_replace_internal_mode = false;
			_reg_request_msg.replace_internal_mode = 0;
			_reg_request_msg.activate_mode_immediately = false;
			snprintf(_reg_request_msg.name, sizeof(_reg_request_msg.name), "mode%d", _registered_mode_num + 1);
			_reg_request_msg.timestamp = hrt_absolute_time();

			_register_ext_component_request_pub.publish(_reg_request_msg);

			_last_register_time = hrt_absolute_time();

			// PX4_INFO("Mode register request Send, request_id: %d",  _registered_mode_num + 1);
		}

		if(_register_ext_component_reply_sub.update(&_reg_reply_msg))
		{
			if(_reg_reply_msg.success)
			{
				_cfg_ctl_sp_msg.flag_armed = true;//这个变量似乎没有用
				_cfg_ctl_sp_msg.flag_multicopter_position_control_enabled = false;
				_cfg_ctl_sp_msg.flag_control_manual_enabled = false;
				_cfg_ctl_sp_msg.flag_control_auto_enabled = false;
				_cfg_ctl_sp_msg.flag_control_offboard_enabled = false;
				_cfg_ctl_sp_msg.flag_control_rates_enabled = false;
				_cfg_ctl_sp_msg.flag_control_attitude_enabled = false;
				_cfg_ctl_sp_msg.flag_control_acceleration_enabled = true;
				_cfg_ctl_sp_msg.flag_control_velocity_enabled = true;
				_cfg_ctl_sp_msg.flag_control_position_enabled = false;
				_cfg_ctl_sp_msg.flag_control_altitude_enabled = true;
				_cfg_ctl_sp_msg.flag_control_climb_rate_enabled = true;
				_cfg_ctl_sp_msg.flag_control_allocation_enabled = true;
				_cfg_ctl_sp_msg.flag_control_termination_enabled = false;
				_cfg_ctl_sp_msg.source_id = _reg_reply_msg.mode_id;
				_cfg_ctl_sp_msg.timestamp = hrt_absolute_time();

				_config_control_setpoints_pub.publish(_cfg_ctl_sp_msg);

				_registered_mode[_registered_mode_num].mode_id = _reg_reply_msg.mode_id;
				_registered_mode[_registered_mode_num].arming_check_id = _reg_reply_msg.arming_check_id;
				_registered_mode_num++;
			}
			PX4_INFO("Mode register reply, success: %d, name: %s, mode id: %d, check id: %d", _reg_reply_msg.success, _reg_reply_msg.name, _reg_reply_msg.mode_id, _reg_reply_msg.arming_check_id);
		}

		if(_arming_check_request_sub.update(&_arming_check_request_msg))
		{
			for(uint8_t i=0; i<_registered_mode_num; i++)
			{
				_arming_check_reply_msg.request_id = _arming_check_request_msg.request_id;
				_arming_check_reply_msg.registration_id = _registered_mode[i].arming_check_id;
				_arming_check_reply_msg.health_component_index = 0;
				_arming_check_reply_msg.health_component_is_present = true;
				_arming_check_reply_msg.health_component_warning = false;
				_arming_check_reply_msg.health_component_error = false;
				_arming_check_reply_msg.can_arm_and_run = true;
				_arming_check_reply_msg.num_events = 0;
				_arming_check_reply_msg.mode_req_angular_velocity = false;
				_arming_check_reply_msg.mode_req_attitude = false;
				_arming_check_reply_msg.mode_req_local_alt = false;
				_arming_check_reply_msg.mode_req_local_position = false;
				_arming_check_reply_msg.mode_req_local_position_relaxed = false;
				_arming_check_reply_msg.mode_req_global_position = false;
				_arming_check_reply_msg.mode_req_mission = false;
				_arming_check_reply_msg.mode_req_home_position = false;
				_arming_check_reply_msg.mode_req_prevent_arming = false;
				_arming_check_reply_msg.mode_req_manual_control = true;//试一下这样能不能实现在出现遥控信号中断时自动处理
				_arming_check_reply_msg.timestamp = hrt_absolute_time();

				if(_registered_mode[i].mode_id == HYDRO_MODE_AUTO_DIVE)
				{
					_arming_check_reply_msg.mode_req_manual_control = false;//自动模式显然不需要检查遥控输入
				}

				_arming_check_reply_pub.publish(_arming_check_reply_msg);

				//PX4_INFO("Arming check reply, arming_check_id: %d", _registered_mode[i].arming_check_id);
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		/*if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_EXTERNAL2)
		{
			//PX4_INFO("Publish SP");
			_att_sp_msg.yaw_body = 0;
			_att_sp_msg.roll_body = -0.2;
			_att_sp_msg.pitch_body = 0;
			_att_sp_msg.thrust_body[0] = 0;
			_att_sp_msg.thrust_body[1] = 0;
			_att_sp_msg.thrust_body[2] = 0;

			const matrix::Quatf q(matrix::Eulerf(_att_sp_msg.roll_body, _att_sp_msg.pitch_body, _att_sp_msg.yaw_body));
			q.copyTo(_att_sp_msg.q_d);

			_vehicle_attitude_setpoint_pub.publish(_att_sp_msg);
		}*/

		parameters_update();
	}

}

void ModeControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int ModeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mode_control_main(int argc, char *argv[])
{
	return ModeControl::main(argc, argv);
}
