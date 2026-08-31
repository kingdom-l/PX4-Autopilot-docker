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

#include "HydroAttitudeControl.hpp"
#include <include/HyModeName.hpp>

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::radians;

HydroAttitudeControl::HydroAttitudeControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
}

HydroAttitudeControl::~HydroAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
HydroAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
HydroAttitudeControl::parameters_update()
{
	// _roll_ctrl.set_time_constant(_param_hy_r_tc.get());
	// _roll_ctrl.set_max_rate(radians(_param_hy_r_rmax.get()));

	// _pitch_ctrl.set_time_constant(_param_hy_p_tc.get());
	// _pitch_ctrl.set_max_rate_pos(radians(_param_hy_p_rmax_pos.get()));
	// _pitch_ctrl.set_max_rate_neg(radians(_param_hy_p_rmax_neg.get()));

	_yaw_ctrl.set_max_rate(radians(_param_hy_y_rmax.get()));

	// _dive_dn_total_time = (hrt_abstime)(1000000.0f * _param_dive_dn_sec.get());
	// _dive_cru_total_time = _dive_dn_total_time + (hrt_abstime)(1000000.0f * _param_dive_cru_sec.get());
	// _dive_up_total_time = _dive_cru_total_time + (hrt_abstime)(1000000.0f * _param_dive_up_sec.get());

	// ****** 更新PID参数 ******
	_roll_pid.update_parameter(_param_hy_r_kp.get(),_param_hy_r_ki.get(),_param_hy_r_maxout.get(),  \
				_param_hy_r_ilimit.get(),_param_hy_r_ea.get(),_param_hy_r_eb.get(),_param_hy_r_fk.get());
	_pitch_pid.update_parameter(_param_hy_p_kp.get(),_param_hy_p_ki.get(),_param_hy_p_maxout.get(),  \
				_param_hy_p_ilimit.get(),_param_hy_p_ea.get(),_param_hy_p_eb.get(),_param_hy_p_fk.get());
	// _yaw_pid.update_parameter(_param_hy_y_kp.get(),_param_hy_y_ki.get(),_param_hy_y_maxout.get(),
	// 			_param_hy_y_ilimit.get(),_param_hy_y_ea.get(),_param_hy_y_eb.get(),_param_hy_y_fk.get());
	// ****** 更新PID参数 ******
}

void
HydroAttitudeControl::vehicle_manual_poll(const float yaw_body)
{
	if (_vhycontrol_mode.flag_control_manual_enabled) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (!_vhycontrol_mode.flag_control_climb_rate_enabled && _vhycontrol_mode.flag_control_attitude_enabled) { // STABILIZED mode generate the attitude setpoint from manual user inputs

				_hy_att_sp.roll_body = _manual_control_setpoint.roll * radians(_param_hy_man_r_max.get());

				_hy_att_sp.pitch_body = -_manual_control_setpoint.pitch * radians(_param_hy_man_p_max.get())
							+ radians(_param_hy_psp_off.get());
				_hy_att_sp.pitch_body = constrain(_hy_att_sp.pitch_body, -radians(_param_hy_man_p_max.get()), radians(_param_hy_man_p_max.get()));

				_hy_att_sp.yaw_body = yaw_body; // yaw is not controlled, so set setpoint to current yaw

				_hy_att_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

				_hy_att_sp.thrust_body[2] = 0.f;
				// printf("att manual \n");

				Quatf q(Eulerf(_hy_att_sp.roll_body, _hy_att_sp.pitch_body, _hy_att_sp.yaw_body));
				q.copyTo(_hy_att_sp.q_d);

				_hy_att_sp.reset_integral = false;

				_hy_att_sp.timestamp = hrt_absolute_time();

				_hy_att_sp_pub.publish(_hy_att_sp);
			}
		}
	}
}

void
HydroAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_hy_att_sp_sub.update(&_hy_att_sp)) {
		_hy_rates_sp.thrust_body[0] = _hy_att_sp.thrust_body[0];
		_hy_rates_sp.thrust_body[1] = _hy_att_sp.thrust_body[1];
		_hy_rates_sp.thrust_body[2] = _hy_att_sp.thrust_body[2];
		// printf("att hello in %f %f\n", (double)_hy_att_sp.thrust_body[0], (double)_hy_att_sp.thrust_body[2]);
	}
	// printf("att hello out %f %f\n", (double)_hy_att_sp.thrust_body[0], (double)_hy_att_sp.thrust_body[2]);
}

float HydroAttitudeControl::get_airspeed_constrained()
{
	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_hy_airspd_trim.get();

	return math::constrain(airspeed, _param_hy_airspd_stall.get(), _param_hy_airspd_max.get());
}

void HydroAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if attitude changed
	if (_att_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) {

		// only update parameters if they changed
		const bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

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

		const matrix::Eulerf euler_angles(_R);

		_vehicle_status_sub.update(&_vehicle_status);
		if(_last_nav_state != _vehicle_status.nav_state)
		{
			_auto_dive_start_time = hrt_absolute_time();
		}

		vehicle_manual_poll(euler_angles.psi());
		// printf("att aft manu_poll: %f %f\n", (double)_hy_att_sp.thrust_body[0], (double)_hy_att_sp.thrust_body[2]);

		vehicle_attitude_setpoint_poll();
		// printf("after att_sp: %f %f %f %f %f\n", (double)_hy_att_sp.thrust_body[0], (double)_hy_att_sp.thrust_body[2], (double)_hy_att_sp.roll_body, (double)_hy_att_sp.pitch_body, (double)_hy_att_sp.yaw_body);

		_vehicle_control_mode_sub.update(&_vhycontrol_mode);
		// printf("att_control_mode: %i %i %i \n", _vhycontrol_mode.flag_control_manual_enabled, _vhycontrol_mode.flag_control_attitude_enabled,
		// 				_vhycontrol_mode.flag_control_rates_enabled);

		if(_vhycontrol_mode.flag_control_rates_enabled){
			if (_hy_att_sp.reset_integral) {
				_hy_rates_sp.reset_integral = true;
			} else {
				_hy_rates_sp.reset_integral = false;
			}


			if (_vhycontrol_mode.flag_control_attitude_enabled) { // STAB/ALTCTL

				/* Run attitude controllers */
				// printf("att control\n");

				if (PX4_ISFINITE(_hy_att_sp.roll_body) && PX4_ISFINITE(_hy_att_sp.pitch_body)) {
					_manual_control_switches_sub.copy(&_manual_control_switches);
					if(_manual_control_switches.arm_switch == 1 && _calibrate_once == 0)//解锁是1，锁定是3
					{
						float roll_pre;
						roll_pre = euler_angles.phi();
						const hrt_abstime time_begin_calib = hrt_absolute_time();

						while(hrt_absolute_time()-time_begin_calib < 3e6 && _roll_calib_en == false)
						{
							_roll_calib_en = true;
							for(int i = 0; i < 20; i++){
								if(fabs(euler_angles.phi() - roll_pre) < 1e-2){
									_roll_bias = euler_angles.phi();
									roll_pre = euler_angles.phi();
								}else{
									_roll_calib_en = false;
									break;
								}

							}

						}
						if(_roll_calib_en == false)
						{
							_roll_bias = 0.f;
						}
						_calibrate_once = 1;
					}
					// _roll_ctrl.control_roll(_hy_att_sp.roll_body, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
					// 			euler_angles.theta());
					// _pitch_ctrl.control_pitch(_hy_att_sp.pitch_body, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
					// 				euler_angles.theta());

					float roll_output = 0.f, pitch_output = 0.f;

					roll_output = _roll_pid.pid_calculate(euler_angles.phi() - _roll_bias, _hy_att_sp.roll_body); // rad
					pitch_output = _pitch_pid.pid_calculate(euler_angles.theta(), _hy_att_sp.pitch_body); // rad
					_yaw_ctrl.control_yaw(_hy_att_sp.roll_body, pitch_output, euler_angles.phi(),
								euler_angles.theta(), get_airspeed_constrained());

					float pitch_body_rate_setpoint = 0.f, roll_body_rate_setpoint = 0.f;
					// 把pitch的惯性角速率转换为机体角速率
					pitch_body_rate_setpoint = cosf(euler_angles.phi()) * pitch_output +
									cosf(euler_angles.theta()) * sinf(euler_angles.phi()) * _yaw_ctrl.get_euler_rate_setpoint();
					pitch_body_rate_setpoint = math::constrain(pitch_body_rate_setpoint, -radians(_param_hy_p_rmax_neg.get()), radians(_param_hy_p_rmax_pos.get()));
					// 把roll轴的惯性角速率转换为机体角速率
					roll_body_rate_setpoint = roll_output - sinf(euler_angles.theta()) * _yaw_ctrl.get_euler_rate_setpoint();
					roll_body_rate_setpoint = math::constrain(roll_body_rate_setpoint, -radians(_param_hy_r_rmax.get()), radians(_param_hy_r_rmax.get()));
					// 把yaw轴的惯性角速率转换为机体角速率
					// yaw_body_rate_setpoint = -sinf(euler_angles.phi()) * pitch_output +
					// 				cosf(euler_angles.phi()) * cosf(euler_angles.theta()) * yaw_output;
					// yaw_body_rate_setpoint = math::constrain(yaw_body_rate_setpoint, -radians(_param_hy_y_rmax.get()), radians(_param_hy_y_rmax.get()));
					/* Update input data for rate controllers */
					Vector3f body_rates_setpoint = Vector3f(roll_body_rate_setpoint, pitch_body_rate_setpoint, _yaw_ctrl.get_body_rate_setpoint());
					// printf("att rate_pitch_sp: %f \n", (double)_hy_rates_sp.pitch);

					/* add yaw rate setpoint from sticks 通过摇杆添加偏航角速率 */
					if (_vhycontrol_mode.flag_control_manual_enabled)
					{
						body_rates_setpoint(2) += math::constrain(_manual_control_setpoint.yaw * radians(_param_man_yr_max.get()),
											-radians(_param_hy_y_rmax.get()), radians(_param_hy_y_rmax.get()));
					}

					/* Publish the rate setpoint for analysis once available */
					_hy_rates_sp.roll = body_rates_setpoint(0);
					_hy_rates_sp.pitch = body_rates_setpoint(1);
					if(_param_hy_y_ctrun_en.get()){
						_hy_rates_sp.yaw = body_rates_setpoint(2);
					}else{
						_hy_rates_sp.yaw = math::constrain(_manual_control_setpoint.yaw * radians(_param_man_yr_max.get()),
											-radians(_param_hy_y_rmax.get()), radians(_param_hy_y_rmax.get()));
					}
					// printf("att roll sp:%f %f e:%f iout:%f %f rate_sp:%f\n", (double)_hy_att_sp.roll_body, (double)euler_angles.phi(), (double)(_hy_att_sp.roll_body - euler_angles.phi()), (double)_roll_pid.pid_get_iout(), (double)roll_output, (double)roll_body_rate_setpoint);
					// printf("att pitch pid_out: %f %f %f %f\n", (double)euler_angles.theta(), (double)_pitch_pid.pid_get_iout(), (double)pitch_output, (double)pitch_body_rate_setpoint);
					// printf("att th_sp: %f %f\n", (double)_hy_att_sp.thrust_body[0], (double)_hy_att_sp.thrust_body[2]);
					// printf("att rate_sp: %f %f %f %f\n", (double)_hy_rates_sp.thrust_body[0], (double)_hy_rates_sp.thrust_body[2], (double)_hy_rates_sp.pitch, (double)_hy_rates_sp.yaw);

					_hy_rates_sp.timestamp = hrt_absolute_time();

					_hy_rates_sp_pub.publish(_hy_rates_sp);
				}

			}
		}else { // MANUAL

		}
		_last_nav_state = _vehicle_status.nav_state;
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int HydroAttitudeControl::task_spawn(int argc, char *argv[])
{
	HydroAttitudeControl *instance = new HydroAttitudeControl();

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

int HydroAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HydroAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
hydro_att_control is the water-air cross medium attitude controller.

)DESCR_STR");

	/*PRINT_MODULE_USAGE_NAME("hydro_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();*/

	return 0;
}

extern "C" __EXPORT int hydro_att_control_main(int argc, char *argv[])
{
	return HydroAttitudeControl::main(argc, argv);
}
