/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include "hydro_pitch_controller.h"
#include "hydro_roll_controller.h"
#include "hydro_yaw_controller.h"
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/landing_gear_wheel.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

using namespace time_literals;

class HydroAttitudeControl final : public ModuleBase<HydroAttitudeControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	HydroAttitudeControl();
	~HydroAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};		/**< vehicle attitude */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};			/**< vehicle attitude setpoint */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};				/**< vehicle status subscription */
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< notification of manual control updates */

	uORB::Publication<vehicle_rates_setpoint_s>	_rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s>	_attitude_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

	vehicle_attitude_setpoint_s		_att_sp{};
	vehicle_rates_setpoint_s		_rates_sp{};
	vehicle_status_s			_vehicle_status{};
	manual_control_setpoint_s		_manual_control_setpoint{};

	matrix::Dcmf _R{matrix::eye<float, 3>()};

	perf_counter_t _loop_perf;

	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HY_AIRSPD_MAX>) _param_hy_airspd_max,		//最大空速
		(ParamFloat<px4::params::HY_AIRSPD_STALL>) _param_hy_airspd_stall,	//失速空速
		(ParamFloat<px4::params::HY_AIRSPD_TRIM>) _param_hy_airspd_trim,	//平衡空速

		(ParamFloat<px4::params::HY_P_RMAX_NEG>) _param_hy_p_rmax_neg,	//机体坐标系下pitch变化率限幅（下限）
		(ParamFloat<px4::params::HY_P_RMAX_POS>) _param_hy_p_rmax_pos,	//机体坐标系下pitch变化率限幅（上限）
		(ParamFloat<px4::params::HY_P_TC>) _param_hy_p_tc,			//pitch控制器比例时间常数（就是比例因子的倒数）

		(ParamFloat<px4::params::HY_R_RMAX>) _param_hy_r_rmax,		//机体坐标系下roll变化率限幅
		(ParamFloat<px4::params::HY_R_TC>) _param_hy_r_tc,			//roll控制器比例时间常数（就是比例因子的倒数）

		(ParamFloat<px4::params::HY_Y_RMAX>) _param_hy_y_rmax,			//机体坐标系下yaw变化率限幅（yaw的变化率直接由当前的roll角和pitch角计算得到，没有反馈控制）

		(ParamFloat<px4::params::HY_PSP_OFF>) _param_hy_psp_off,		//pitch杆量为0时的pitch偏移量
		(ParamFloat<px4::params::HY_MAN_P_MAX>) _param_hy_man_p_max,		//pitch杆量为最大时的pitch角度
		(ParamFloat<px4::params::HY_MAN_R_MAX>) _param_hy_man_r_max,		//roll杆量为最大时的roll角度
		(ParamFloat<px4::params::HY_MAN_YR_MAX>) _param_man_yr_max,		//yaw杆量为最大时的yaw变化率

		(ParamFloat<px4::params::HY_DIVE_DN_DEG>) _param_dive_dn_deg,		//自动潜行（下潜）的角度
		(ParamFloat<px4::params::HY_DIVE_DN_THR>) _param_dive_dn_thr,		//自动潜行（下潜）的推力
		(ParamFloat<px4::params::HY_DIVE_DN_SEC>) _param_dive_dn_sec,		//自动潜行（下潜）的时间
		(ParamFloat<px4::params::HY_DIVE_DN_DEG>) _param_dive_cru_deg,		//自动潜行（航行）
		(ParamFloat<px4::params::HY_DIVE_DN_THR>) _param_dive_cru_thr,
		(ParamFloat<px4::params::HY_DIVE_DN_SEC>) _param_dive_cru_sec,
		(ParamFloat<px4::params::HY_DIVE_DN_DEG>) _param_dive_up_deg,		//自动潜行（上浮）
		(ParamFloat<px4::params::HY_DIVE_DN_THR>) _param_dive_up_thr,
		(ParamFloat<px4::params::HY_DIVE_DN_SEC>) _param_dive_up_sec

	)

	RollController _roll_ctrl;
	PitchController _pitch_ctrl;
	YawController _yaw_ctrl;

	hrt_abstime _dive_dn_total_time;
	hrt_abstime _dive_cru_total_time;
	hrt_abstime _dive_up_total_time;

	hrt_abstime _auto_dive_start_time{0};
	uint8_t _last_nav_state{255};

	void parameters_update();
	void auto_dive_poll(const float yaw_body);
	void vehicle_manual_poll(const float yaw_body);
	void vehicle_attitude_setpoint_poll();
	float get_airspeed_constrained();
};
