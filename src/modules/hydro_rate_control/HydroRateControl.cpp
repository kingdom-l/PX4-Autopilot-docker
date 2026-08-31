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
	_hy_rollr_lpf.T = 1.0f/_param_hy_rr_lpf_fs.get();
	_hy_rollr_lpf.fc = _param_hy_rr_lpf_fc.get();
	_hy_rollr_lpf.alpha = 2.0f * 3.14159f * _hy_rollr_lpf.fc * _hy_rollr_lpf.T;
	_hy_rollr_lpf.alpha = _hy_rollr_lpf.alpha / (_hy_rollr_lpf.alpha + 1.0f);
	_hy_rollr_lpf.out = 0.0f;

	_hy_yawr_lpf.T = 1.0f/_param_hy_yr_lpf_fs.get();
	_hy_yawr_lpf.fc = _param_hy_yr_lpf_fc.get();
	_hy_yawr_lpf.alpha = 2.0f * 3.14159f * _hy_yawr_lpf.fc * _hy_yawr_lpf.T;
	_hy_yawr_lpf.alpha = _hy_yawr_lpf.alpha / (_hy_yawr_lpf.alpha + 1.0f);
	_hy_yawr_lpf.out = 0.0f;

	// const Vector3f rate_p = Vector3f(_param_hy_rr_p.get(), _param_hy_pr_p.get(), _param_hy_yr_p.get());
	// const Vector3f rate_i = Vector3f(_param_hy_rr_i.get(), _param_hy_pr_i.get(), _param_hy_yr_i.get());
	// const Vector3f rate_d = Vector3f(_param_hy_rr_d.get(), _param_hy_pr_d.get(), _param_hy_yr_d.get());

	// _rate_control.setPidGains(rate_p, rate_i, rate_d);

	// _rate_control.setIntegratorLimit(
	// 	Vector3f(_param_hy_rr_imax.get(), _param_hy_pr_imax.get(), _param_hy_yr_imax.get()));

	// _rate_control.setFeedForwardGain(
	// 	// set FF gains to 0 as we add the FF control outside of the rate controller
	// 	Vector3f(0.f, 0.f, 0.f));

	_ratex_pid.update_parameter(_param_hy_rr_kp.get(),_param_hy_rr_ki.get(),_param_hy_rr_maxout.get(),  \
				_param_hy_rr_ilimit.get(),_param_hy_rr_ea.get(),_param_hy_rr_eb.get(),_param_hy_rr_fk.get());
	_ratey_pid.update_parameter(_param_hy_pr_kp.get(),_param_hy_pr_ki.get(),_param_hy_pr_maxout.get(),  \
				_param_hy_pr_ilimit.get(),_param_hy_pr_ea.get(),_param_hy_pr_eb.get(),_param_hy_pr_fk.get());
	_ratez_pid.update_parameter(_param_hy_yr_kp.get(),_param_hy_yr_ki.get(),_param_hy_yr_maxout.get(),
				_param_hy_yr_ilimit.get(),_param_hy_yr_ea.get(),_param_hy_yr_eb.get(),_param_hy_yr_fk.get());


	return PX4_OK;
}

void
HydroRateControl::vehicle_manual_poll()
{
	if (_vhycontrol_mode.flag_control_manual_enabled) {

		// printf("here4 ");

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (_vhycontrol_mode.flag_control_rates_enabled &&
			    !_vhycontrol_mode.flag_control_attitude_enabled) { // ACRO

				_rates_sp.roll = _manual_control_setpoint.roll * radians(_param_hy_acro_x_max.get()); // _manual_control_setpoint.roll取值为[-1, 1]
				_rates_sp.yaw = _manual_control_setpoint.yaw * radians(_param_hy_acro_z_max.get());
				_rates_sp.pitch = -_manual_control_setpoint.pitch * radians(_param_hy_acro_y_max.get()); // hy_acro_y_max: Acro body pitch max rate setpoint 90
				_rates_sp.timestamp = hrt_absolute_time();
				_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;
				_rates_sp.thrust_body[2] = 0.f;
				// printf("rate_manual acro: %f %f\n", (double)_rates_sp.thrust_body[0], (double)_rates_sp.thrust_body[2]);

				_rate_sp_pub.publish(_rates_sp);

			} else { // _vehicle_status.nav_state == HYDRO_MODE_MANUAL/ HYDRO_MODE_ALTCTL / STABLIZED

				_hydro_torque_setpoint.xyz[0] = math::constrain(_manual_control_setpoint.roll * _param_hy_man_r_sc.get() +
								  _param_trim_roll.get(), -1.f, 1.f);
				_hydro_torque_setpoint.xyz[1] = math::constrain(-_manual_control_setpoint.pitch * _param_hy_man_p_sc.get() +
								  _param_trim_pitch.get(), -1.f, 1.f); // manual_pitch:[-1, 1]
				_hydro_torque_setpoint.xyz[2] = math::constrain(_manual_control_setpoint.yaw * _param_hy_man_y_sc.get() +
								  _param_trim_yaw.get(), -1.f, 1.f);

				_hydro_thrust_setpoint.xyz[0] = math::constrain((_manual_control_setpoint.throttle + 1.f) * .5f, 0.f, 1.f);

				_hydro_thrust_setpoint.xyz[2] = 0.f;
				// printf("here6 : %f %f ", (double)_hydro_torque_setpoint.xyz[0], (double)_hydro_torque_setpoint.xyz[1]);
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

void HydroRateControl::LPFilter(float in, LowPassFilter* lpf_params)
{
	lpf_params->out += lpf_params->alpha * (in - lpf_params->out);
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

		// printf("here1 ");

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
			// printf("here2 ");
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
			// printf("here3 ");
		}

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);

		Vector3f rates(angular_velocity.xyz);
		Vector3f angular_accel{angular_velocity.xyz_derivative};

		_vehicle_status_sub.update(&_vehicle_status);

		_vehicle_control_mode_sub.update(&_vhycontrol_mode);

		vehicle_manual_poll();

		if (_vhycontrol_mode.flag_control_rates_enabled) { // STAB/ACRO/ALT

			// printf("here7 ");

			const float airspeed = get_airspeed_and_update_scaling(); //15
			// printf("airspd: %f ", (double)airspeed);

			/* reset integrals where needed */
			// if (_rates_sp.reset_integral) {
			// 	_rate_control.resetIntegral();
			// 	// PX4_INFO("hello here");
			// }

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
						       0.0f); // hy_dtrim_y_vmin zero

			} else {
				trim(0) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
						       _param_hy_dtrim_r_vmax.get());
				trim(1) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
						       _param_hy_dtrim_p_vmax.get());
				trim(2) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
						       _param_hy_dtrim_y_vmax.get()); // hy_dtrim_y_vmax zero
				// printf("here8 ");
			}

			_hy_rates_sp_sub.update(&_rates_sp);
			// printf("rt _rates_sp: %f, %f\n", (double)_rates_sp.thrust_body[0], (double)_rates_sp.thrust_body[2]);

			if(_param_hy_rr_lpf_en.get()){
				LPFilter(rates(0), &_hy_rollr_lpf);
				rates(0) = _hy_rollr_lpf.out;
			}
			if(_param_hy_yr_lpf_en.get()){
				LPFilter(rates(2), &_hy_yawr_lpf);
				rates(2) = _hy_yawr_lpf.out;
			}

			// Vector3f body_rates_setpoint = Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw);

			// Run attitude RATE controllers which need the desired attitudes from above, add trim.
			float ratex_output = 0.f, ratey_output = 0.f, ratez_output = 0.f;

			ratex_output = _ratex_pid.pid_calculate(rates(0), _rates_sp.roll); // rad
			ratey_output = _ratey_pid.pid_calculate(rates(1), _rates_sp.pitch); // rad
			ratez_output = _ratez_pid.pid_calculate(rates(2), _rates_sp.yaw); // rad
			// printf("rate_pid out: %f %f %f \n", (double)ratex_output, (double)ratey_output, (double)ratez_output);

			const Vector3f angular_acceleration_setpoint = Vector3f(ratex_output, ratey_output, ratez_output);

			// Vector3f gain_ff(0, 0, 0);
			// const Vector3f feedforward = gain_ff.emult(body_rates_setpoint) * _airspeed_scaling;
			// PX4_INFO("ff: %f, %f, %f", (double)feedforward(0), (double)feedforward(1), (double)feedforward(2));

			Vector3f control_u = angular_acceleration_setpoint * _airspeed_scaling * _airspeed_scaling;

			// Special case yaw in Acro: if the parameter HY_ACRO_YAW_CTL is not set then don't control yaw
			if (!_vhycontrol_mode.flag_control_attitude_enabled && !_param_hy_acro_yaw_en.get()) { // HY_ACRO_YAW_EN默认为0
				control_u(2) = _manual_control_setpoint.yaw * _param_hy_man_y_sc.get(); // HY_MAN_Y_SC: manual yaw scale:1
				// _rate_control.resetIntegral(2);
				// printf("here9 ");
			}

			control_u(1) = control_u(1) - _param_hy_pr_tcp.get();

			// PX4_INFO("control_u: %f, %f, %f", (double)control_u(0), (double)control_u(1), (double)control_u(2));
			if (control_u.isAllFinite()) {
				matrix::constrain(control_u + trim, -1.f, 1.f).copyTo(_hydro_torque_setpoint.xyz);
				// printf("here10 : %f %f %f ", (double)control_u(0), (double)control_u(1), (double)control_u(2));
				// printf("hy_torque: %f %f \n", (double)_hydro_torque_setpoint.xyz[0], (double)_hydro_torque_setpoint.xyz[1]);
			} else {
				// _rate_control.resetIntegral();
				trim.copyTo(_hydro_torque_setpoint.xyz);
			}

			/* throttle passed through if it is finite */
			_hydro_thrust_setpoint.xyz[0] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ? _rates_sp.thrust_body[0] : 0.0f;
			_hydro_thrust_setpoint.xyz[2] = PX4_ISFINITE(_rates_sp.thrust_body[2]) ? _rates_sp.thrust_body[2] : 0.0f;
			// printf("rt here11: %f, %f\n", (double)_hydro_thrust_setpoint.xyz[0], (double)_hydro_thrust_setpoint.xyz[2]); // 油门量[0, 1]
			// printf("hy_torque: %f %f %f \n", (double)_hydro_torque_setpoint.xyz[0], (double)_hydro_torque_setpoint.xyz[1], (double)_hydro_torque_setpoint.xyz[2]);

			/* scale effort by battery status */
			if (_param_hy_bat_scale_en.get() && _hydro_thrust_setpoint.xyz[0] > 0.1f) {

				if (_battery_status_sub.updated()) {
					battery_status_s battery_status{};

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_scale = battery_status.scale;
					}
				}
				// printf("here12 ");
				_hydro_thrust_setpoint.xyz[0] *= _battery_scale;
			}

			// ****** 调试角速率控制，与postion通道显示信息冲突 ******
			// vehicle_attitude_s att{};
			// matrix::Dcmf _R{matrix::eye<float, 3>()};
			// _att_sub.copy(&att);
			// _R = matrix::Quatf(att.q);
			// matrix::Eulerf euler_angles(_R);
			// _rate_pos_sp.timestamp = hrt_absolute_time();
			// _rate_pos_sp.x = 0;
			// _rate_pos_sp.y = euler_angles.phi();
			// _rate_pos_sp.z = _rates_sp.roll;
			// _rate_pos_sp.vx = rates(0);
			// _rate_pos_sp.vy = _rates_sp.yaw;
			// _rate_pos_sp.vz = rates(2);
			// _rate_pos_sp_pub.publish(_rate_pos_sp);
			// ****** 调试角速率控制，与postion通道显示信息冲突 ******

			// printf("ratex sp:%f %f e:%f e_i:%f u_sp:%f\n", (double)_rates_sp.roll, (double)rates(0), (double)(_rates_sp.roll - rates(0)), (double)_ratex_pid.pid_get_iout(), (double)control_u(0));
			// printf("ratey sp:%f %f e:%f e_i:%f u_sp:%f\n", (double)_rates_sp.pitch, (double)rates(1), (double)(_rates_sp.pitch - rates(1)), (double)_ratey_pid.pid_get_iout(), (double)control_u(1));

		} else { // MANUAL
			// _rate_control.resetIntegral();
		}

		if (_vhycontrol_mode.flag_control_rates_enabled ||
		    _vhycontrol_mode.flag_control_attitude_enabled ||
		    _vhycontrol_mode.flag_control_manual_enabled){
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
			// printf("here13 \n");
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
