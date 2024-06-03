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
	_roll_ctrl.set_time_constant(_param_hy_r_tc.get());
	_roll_ctrl.set_max_rate(radians(_param_hy_r_rmax.get()));

	_pitch_ctrl.set_time_constant(_param_hy_p_tc.get());
	_pitch_ctrl.set_max_rate_pos(radians(_param_hy_p_rmax_pos.get()));
	_pitch_ctrl.set_max_rate_neg(radians(_param_hy_p_rmax_neg.get()));

	_yaw_ctrl.set_max_rate(radians(_param_hy_y_rmax.get()));

	_dive_dn_total_time = (hrt_abstime)(1000000.0f * _param_dive_dn_sec.get());
	_dive_cru_total_time = _dive_dn_total_time + (hrt_abstime)(1000000.0f * _param_dive_cru_sec.get());
	_dive_up_total_time = _dive_cru_total_time + (hrt_abstime)(1000000.0f * _param_dive_up_sec.get());
}

void
HydroAttitudeControl::auto_dive_poll(const float yaw_body)
{
	if (_vehicle_status.nav_state == HYDRO_MODE_AUTO_DIVE) {

		_att_sp.roll_body = 0;
		_att_sp.yaw_body = yaw_body;

		if(hrt_absolute_time() - _auto_dive_start_time < _dive_dn_total_time)
		{
			_att_sp.pitch_body = radians(_param_dive_dn_deg.get());
			_att_sp.thrust_body[0] = _param_dive_dn_thr.get();
		}
		else if(hrt_absolute_time() - _auto_dive_start_time < _dive_cru_total_time)
		{
			_att_sp.pitch_body = radians(_param_dive_cru_deg.get());
			_att_sp.thrust_body[0] = _param_dive_cru_thr.get();
		}
		else if(hrt_absolute_time() - _auto_dive_start_time < _dive_up_total_time)
		{
			_att_sp.pitch_body = radians(_param_dive_up_deg.get());
			_att_sp.thrust_body[0] = _param_dive_up_thr.get();
		}
		else
		{
			_att_sp.pitch_body = radians(0);
			_att_sp.thrust_body[0] = 0;
		}

		Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
		q.copyTo(_att_sp.q_d);

		_att_sp.reset_integral = false;

		_att_sp.timestamp = hrt_absolute_time();

		_attitude_sp_pub.publish(_att_sp);
	}
}

void
HydroAttitudeControl::vehicle_manual_poll(const float yaw_body)
{
	if (_vehicle_status.nav_state == HYDRO_MODE_STABILIZED) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			_att_sp.roll_body = _manual_control_setpoint.roll * radians(_param_hy_man_r_max.get());

			_att_sp.pitch_body = -_manual_control_setpoint.pitch * radians(_param_hy_man_p_max.get())
						+ radians(_param_hy_psp_off.get());
			_att_sp.pitch_body = constrain(_att_sp.pitch_body, -radians(_param_hy_man_p_max.get()), radians(_param_hy_man_p_max.get()));

			_att_sp.yaw_body = yaw_body; // yaw is not controlled, so set setpoint to current yaw
			_att_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

			Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
			q.copyTo(_att_sp.q_d);

			_att_sp.reset_integral = false;

			_att_sp.timestamp = hrt_absolute_time();

			_attitude_sp_pub.publish(_att_sp);
		}
	}
}

void
HydroAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
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

		auto_dive_poll(euler_angles.psi());

		vehicle_manual_poll(euler_angles.psi());

		vehicle_attitude_setpoint_poll();

		if (_vehicle_status.nav_state == HYDRO_MODE_STABILIZED || _vehicle_status.nav_state == HYDRO_MODE_AUTO_DIVE ) {

			if (_att_sp.reset_integral) {
				_rates_sp.reset_integral = true;
			} else {
				_rates_sp.reset_integral = false;
			}

			/* Run attitude controllers */

			if (PX4_ISFINITE(_att_sp.roll_body) && PX4_ISFINITE(_att_sp.pitch_body)) {
				_roll_ctrl.control_roll(_att_sp.roll_body, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
							euler_angles.theta());
				_pitch_ctrl.control_pitch(_att_sp.pitch_body, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
								euler_angles.theta());
				_yaw_ctrl.control_yaw(_att_sp.roll_body, _pitch_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
							euler_angles.theta(), get_airspeed_constrained());

				/* Update input data for rate controllers */
				Vector3f body_rates_setpoint = Vector3f(_roll_ctrl.get_body_rate_setpoint(), _pitch_ctrl.get_body_rate_setpoint(),
									_yaw_ctrl.get_body_rate_setpoint());

				/* add yaw rate setpoint from sticks */
				if (_vehicle_status.nav_state == HYDRO_MODE_STABILIZED)
				{
					body_rates_setpoint(2) += math::constrain(_manual_control_setpoint.yaw * radians(_param_man_yr_max.get()),
										  -radians(_param_hy_y_rmax.get()), radians(_param_hy_y_rmax.get()));
				}

				/* Publish the rate setpoint for analysis once available */
				_rates_sp.roll = body_rates_setpoint(0);
				_rates_sp.pitch = body_rates_setpoint(1);
				_rates_sp.yaw = body_rates_setpoint(2);

				_rates_sp.timestamp = hrt_absolute_time();

				_rate_sp_pub.publish(_rates_sp);
			}

		} else {

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
