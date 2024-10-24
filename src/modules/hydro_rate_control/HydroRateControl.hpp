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

#pragma once

#include <lib/rate_control/rate_control.hpp>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

using namespace time_literals;

class HydroRateControl final : public ModuleBase<HydroRateControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	HydroRateControl();
	~HydroRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<vehicle_rates_setpoint_s>	_rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_hydro_torque_setpoint_pub{ORB_ID(hydro_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_hydro_thrust_setpoint_pub{ORB_ID(hydro_thrust_setpoint)};

	manual_control_setpoint_s		_manual_control_setpoint{};
	vehicle_thrust_setpoint_s		_hydro_thrust_setpoint{};
	vehicle_torque_setpoint_s		_hydro_torque_setpoint{};
	vehicle_rates_setpoint_s		_rates_sp{};
	vehicle_status_s			_vehicle_status{};

	perf_counter_t _loop_perf;

	hrt_abstime _last_run{0};

	float _airspeed_scaling{1.0f};

	float _battery_scale{1.0f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HY_AIRSPD_MAX>) _param_hy_airspd_max,		//最大空速
		(ParamFloat<px4::params::HY_AIRSPD_MIN>) _param_hy_airspd_min,		//最小空速
		(ParamFloat<px4::params::HY_AIRSPD_STALL>) _param_hy_airspd_stall,	//失速空速
		(ParamFloat<px4::params::HY_AIRSPD_TRIM>) _param_hy_airspd_trim,	//平衡空速

		(ParamFloat<px4::params::HY_ACRO_X_MAX>) _param_hy_acro_x_max,		//ACRO模式下各轴杆量最大时对应的角度变化率
		(ParamFloat<px4::params::HY_ACRO_Y_MAX>) _param_hy_acro_y_max,
		(ParamFloat<px4::params::HY_ACRO_Z_MAX>) _param_hy_acro_z_max,
		(ParamInt<px4::params::HY_ACRO_YAW_EN>) _param_hy_acro_yaw_en,		//ACRO模式下是否启用YAW轴控制（不启用则yaw杆量映射到torque上）

		(ParamInt<px4::params::HY_ARSP_SCALE_EN>) _param_hy_arsp_scale_en,	//启用空速系数（根据空速对控制器进行补偿）

		(ParamBool<px4::params::HY_BAT_SCALE_EN>) _param_hy_bat_scale_en,	//启用电池系数（应该是当电池输出能力降低时，对推力输出进行放大）

		(ParamFloat<px4::params::HY_DTRIM_P_VMAX>) _param_hy_dtrim_p_vmax,	//最大空速下相较于平衡空速还需要额外补偿多少pitch力矩才能使让pitch轴不动
		(ParamFloat<px4::params::HY_DTRIM_P_VMIN>) _param_hy_dtrim_p_vmin,
		(ParamFloat<px4::params::HY_DTRIM_R_VMAX>) _param_hy_dtrim_r_vmax,
		(ParamFloat<px4::params::HY_DTRIM_R_VMIN>) _param_hy_dtrim_r_vmin,
		(ParamFloat<px4::params::HY_DTRIM_Y_VMAX>) _param_hy_dtrim_y_vmax,
		(ParamFloat<px4::params::HY_DTRIM_Y_VMIN>) _param_hy_dtrim_y_vmin,

		(ParamFloat<px4::params::HY_MAN_P_SC>) _param_hy_man_p_sc,		//MANUAL模式下各轴杆量最大时对应的torque
		(ParamFloat<px4::params::HY_MAN_R_SC>) _param_hy_man_r_sc,
		(ParamFloat<px4::params::HY_MAN_Y_SC>) _param_hy_man_y_sc,

		(ParamFloat<px4::params::HY_PR_FF>) _param_hy_pr_ff,			//pitch rate控制器的前馈、ki、imax、kp、kd
		(ParamFloat<px4::params::HY_PR_I>) _param_hy_pr_i,
		(ParamFloat<px4::params::HY_PR_IMAX>) _param_hy_pr_imax,
		(ParamFloat<px4::params::HY_PR_P>) _param_hy_pr_p,
		(ParamFloat<px4::params::HY_PR_D>) _param_hy_pr_d,

		(ParamFloat<px4::params::HY_RLL_TO_YAW_FF>) _param_hy_rll_to_yaw_ff,	//将roll轴力矩直接前馈到yaw轴上
		(ParamFloat<px4::params::HY_RR_FF>) _param_hy_rr_ff,
		(ParamFloat<px4::params::HY_RR_I>) _param_hy_rr_i,
		(ParamFloat<px4::params::HY_RR_IMAX>) _param_hy_rr_imax,
		(ParamFloat<px4::params::HY_RR_P>) _param_hy_rr_p,
		(ParamFloat<px4::params::HY_RR_D>) _param_hy_rr_d,

		(ParamFloat<px4::params::HY_YR_FF>) _param_hy_yr_ff,
		(ParamFloat<px4::params::HY_YR_I>) _param_hy_yr_i,
		(ParamFloat<px4::params::HY_YR_IMAX>) _param_hy_yr_imax,
		(ParamFloat<px4::params::HY_YR_P>) _param_hy_yr_p,
		(ParamFloat<px4::params::HY_YR_D>) _param_hy_yr_d,

		(ParamFloat<px4::params::TRIM_PITCH>) _param_trim_pitch,		//平衡空速下需要补偿多少pitch力矩才能使让pitch轴不动
		(ParamFloat<px4::params::TRIM_ROLL>) _param_trim_roll,
		(ParamFloat<px4::params::TRIM_YAW>) _param_trim_yaw,

		(ParamFloat<px4::params::HY_THR_TO_PIT_FF>) _param_thr_to_pit_ff	//推力前馈到pit轴力矩上
	)

	RateControl _rate_control; ///< class for rate control calculations


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	float 		get_airspeed_and_update_scaling();
	void		vehicle_manual_poll();
};
