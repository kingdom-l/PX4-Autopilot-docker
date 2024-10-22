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

#include "HydroRateControl.hpp"

#include <include/HyModeName.hpp>

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

HydroRateControl::HydroRateControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
}

HydroRateControl::~HydroRateControl()
{
	perf_free(_loop_perf);
}

bool
HydroRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
HydroRateControl::parameters_update()
{
	const Vector3f rate_p = Vector3f(_param_hy_rr_p.get(), _param_hy_pr_p.get(), _param_hy_yr_p.get());
	const Vector3f rate_i = Vector3f(_param_hy_rr_i.get(), _param_hy_pr_i.get(), _param_hy_yr_i.get());
	const Vector3f rate_d = Vector3f(_param_hy_rr_d.get(), _param_hy_pr_d.get(), _param_hy_yr_d.get());

	_rate_control.setPidGains(rate_p, rate_i, rate_d);

	_rate_control.setIntegratorLimit(
		Vector3f(_param_hy_rr_imax.get(), _param_hy_pr_imax.get(), _param_hy_yr_imax.get()));

	_rate_control.setFeedForwardGain(
		// set FF gains to 0 as we add the FF control outside of the rate controller
		Vector3f(0.f, 0.f, 0.f));

	return PX4_OK;
}

void
HydroRateControl::vehicle_manual_poll()
{
	if (_vehicle_status.nav_state == HYDRO_MODE_ACRO || _vehicle_status.nav_state == HYDRO_MODE_MANUAL) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (_vehicle_status.nav_state == HYDRO_MODE_ACRO) {

				_rates_sp.roll = _manual_control_setpoint.roll * radians(_param_hy_acro_x_max.get()); // _manual_control_setpoint.roll取值为[-1, 1]
				_rates_sp.yaw = _manual_control_setpoint.yaw * radians(_param_hy_acro_z_max.get());
				_rates_sp.pitch = -_manual_control_setpoint.pitch * radians(_param_hy_acro_y_max.get());
				_rates_sp.timestamp = hrt_absolute_time();
				_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

				_rate_sp_pub.publish(_rates_sp);

			} else { // _vehicle_status.nav_state == HYDRO_MODE_MANUAL

				_hydro_torque_setpoint.xyz[0] = math::constrain(_manual_control_setpoint.roll * _param_hy_man_r_sc.get() +
								  _param_trim_roll.get(), -1.f, 1.f);
				_hydro_torque_setpoint.xyz[1] = math::constrain(-_manual_control_setpoint.pitch * _param_hy_man_p_sc.get() +
								  _param_trim_pitch.get(), -1.f, 1.f);
				_hydro_torque_setpoint.xyz[2] = math::constrain(_manual_control_setpoint.yaw * _param_hy_man_y_sc.get() +
								  _param_trim_yaw.get(), -1.f, 1.f);

				_hydro_thrust_setpoint.xyz[0] = math::constrain((_manual_control_setpoint.throttle + 1.f) * .5f, 0.f, 1.f);
			}
		}
	}
}

float HydroRateControl::get_airspeed_and_update_scaling()
{
	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_hy_airspd_trim.get();

	const float airspeed_constrained = constrain(constrain(airspeed, _param_hy_airspd_stall.get(),
					   _param_hy_airspd_max.get()), 0.1f, 1000.0f);

	_airspeed_scaling = (_param_hy_arsp_scale_en.get()) ? (_param_hy_airspd_trim.get() / airspeed_constrained) : 1.0f;

	return airspeed_constrained;
}

void HydroRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

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

		vehicle_angular_velocity_s vehicle_angular_velocity{};

		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
			dt = math::constrain((vehicle_angular_velocity.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = vehicle_angular_velocity.timestamp_sample;
			// PX4_INFO("dt: %f", (double)dt);
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);

		Vector3f rates(angular_velocity.xyz);
		Vector3f angular_accel{angular_velocity.xyz_derivative};

		_vehicle_status_sub.update(&_vehicle_status);

		vehicle_manual_poll();

		if (_vehicle_status.nav_state == HYDRO_MODE_STABILIZED || _vehicle_status.nav_state == HYDRO_MODE_AUTO_DIVE || _vehicle_status.nav_state == HYDRO_MODE_ACRO) {

			const float airspeed = get_airspeed_and_update_scaling(); //15

			/* reset integrals where needed */
			if (_rates_sp.reset_integral) {
				_rate_control.resetIntegral();
				// PX4_INFO("hello here");
			}

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			Vector3f trim(_param_trim_roll.get(), _param_trim_pitch.get(), _param_trim_yaw.get());

			if (airspeed < _param_hy_airspd_trim.get()) {
				trim(0) += interpolate(airspeed, _param_hy_airspd_min.get(), _param_hy_airspd_trim.get(),
						       _param_hy_dtrim_r_vmin.get(),
						       0.0f);
				trim(1) += interpolate(airspeed, _param_hy_airspd_min.get(), _param_hy_airspd_trim.get(),
						       _param_hy_dtrim_p_vmin.get(),
						       0.0f);
				trim(2) += interpolate(airspeed, _param_hy_airspd_min.get(), _param_hy_airspd_trim.get(),
						       _param_hy_dtrim_y_vmin.get(),
						       0.0f);

			} else {
				trim(0) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
						       _param_hy_dtrim_r_vmax.get());
				trim(1) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
						       _param_hy_dtrim_p_vmax.get());
				trim(2) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
						       _param_hy_dtrim_y_vmax.get());
			}

			_rates_sp_sub.update(&_rates_sp);

			Vector3f body_rates_setpoint = Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw);

			// Run attitude RATE controllers which need the desired attitudes from above, add trim.
			const Vector3f angular_acceleration_setpoint = _rate_control.update(rates, body_rates_setpoint, angular_accel, dt,
					false);

			// const Vector3f gain_ff(_param_hy_rr_ff.get(), _param_hy_pr_ff.get(), _param_hy_yr_ff.get());
			Vector3f gain_ff(0, 0, 0);
			const Vector3f feedforward = gain_ff.emult(body_rates_setpoint) * _airspeed_scaling;
			// PX4_INFO("ff: %f, %f, %f", (double)feedforward(0), (double)feedforward(1), (double)feedforward(2));

			Vector3f control_u = angular_acceleration_setpoint * _airspeed_scaling * _airspeed_scaling + feedforward;

			// Special case yaw in Acro: if the parameter HY_ACRO_YAW_CTL is not set then don't control yaw
			if (_vehicle_status.nav_state == HYDRO_MODE_ACRO && !_param_hy_acro_yaw_en.get()) { // HY_ACRO_YAW_EN默认为0
				control_u(2) = _manual_control_setpoint.yaw * _param_hy_man_y_sc.get(); // HY_MAN_Y_SC: manual yaw scale
				_rate_control.resetIntegral(2);
			}

			// PX4_INFO("control_u: %f, %f, %f", (double)control_u(0), (double)control_u(1), (double)control_u(2));
			if (control_u.isAllFinite()) {
				matrix::constrain(control_u + trim, -1.f, 1.f).copyTo(_hydro_torque_setpoint.xyz);

			} else {
				_rate_control.resetIntegral();
				trim.copyTo(_hydro_torque_setpoint.xyz);
			}

			/* throttle passed through if it is finite */
			_hydro_thrust_setpoint.xyz[0] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ? _rates_sp.thrust_body[0] : 0.0f;

			/* scale effort by battery status */
			if (_param_hy_bat_scale_en.get() && _hydro_thrust_setpoint.xyz[0] > 0.1f) {

				if (_battery_status_sub.updated()) {
					battery_status_s battery_status{};

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_scale = battery_status.scale;
					}
				}

				_hydro_thrust_setpoint.xyz[0] *= _battery_scale;
			}

		} else {
			_rate_control.resetIntegral();
		}

		if (_vehicle_status.nav_state == HYDRO_MODE_STABILIZED || _vehicle_status.nav_state == HYDRO_MODE_AUTO_DIVE ||
			_vehicle_status.nav_state == HYDRO_MODE_ACRO || _vehicle_status.nav_state == HYDRO_MODE_MANUAL){
			// Add feed-forward from roll control output to yaw control output
			// This can be used to counteract the adverse yaw effect when rolling the plane
			_hydro_torque_setpoint.xyz[2] = math::constrain(_hydro_torque_setpoint.xyz[2] + _param_hy_rll_to_yaw_ff.get() *
							_hydro_torque_setpoint.xyz[0], -1.f, 1.f);

			//推力前馈到pit轴力矩上
			_hydro_torque_setpoint.xyz[1] = math::constrain(_hydro_torque_setpoint.xyz[1] + _param_thr_to_pit_ff.get() *
							_hydro_thrust_setpoint.xyz[0], -1.f, 1.f);

			_hydro_thrust_setpoint.timestamp = hrt_absolute_time();
			_hydro_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			_hydro_thrust_setpoint_pub.publish(_hydro_thrust_setpoint);

			_hydro_torque_setpoint.timestamp = hrt_absolute_time();
			_hydro_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			_hydro_torque_setpoint_pub.publish(_hydro_torque_setpoint);
		}
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int HydroRateControl::task_spawn(int argc, char *argv[])
{
	HydroRateControl *instance = new HydroRateControl();

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

int HydroRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HydroRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
hydro_rate_control is the water-air cross medium rate controller.

)DESCR_STR");

	/*PRINT_MODULE_USAGE_NAME("hydro_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();*/

	return 0;
}

extern "C" __EXPORT int hydro_rate_control_main(int argc, char *argv[])
{
	return HydroRateControl::main(argc, argv);
}
