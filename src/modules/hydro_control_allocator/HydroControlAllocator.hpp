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

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace matrix;

class HydroControlAllocator : public ModuleBase<HydroControlAllocator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:

	HydroControlAllocator();
	~HydroControlAllocator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:
	uORB::SubscriptionCallbackWorkItem 	_hydro_torque_setpoint_sub{this, ORB_ID(hydro_torque_setpoint)};
	uORB::SubscriptionCallbackWorkItem 	_hydro_thrust_setpoint_sub{this, ORB_ID(hydro_thrust_setpoint)};

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	manual_control_setpoint_s		_manual_control_setpoint{0};
	vehicle_attitude_s			_vehicle_attitude{0};

	uORB::Publication<actuator_motors_s>	_hydro_motors_pub{ORB_ID(hydro_motors)};
	uORB::Publication<actuator_servos_s>	_hydro_servos_pub{ORB_ID(hydro_servos)};


	struct StructureInfo{
		float x2;
		float y2;
		float z2;
		float yT;
		float yh;
		float xe;
		bool with_thrust;
	};
	StructureInfo _st_info;
	matrix::Matrix<float, 5, 6> _hy_effectiveness;
	matrix::Matrix<float, 6, 5> _hy_mix;
	matrix::Vector<float, 5> _wrench_sp;

	struct NfParams{
		float Cl;
		float Cl0;
		float Cd;
		float Cd0;
		float S_wing;
		float Fx;
		float Fz;
	};
	float _Va2;
	float _rho = 1e3;
	NfParams _nf_params_hy_wr;
	NfParams _nf_params_hy_wl;
	NfParams _nf_params_hy_htail;

	Vector2f optim(float x_opt[2], NfParams p);
	SquareMatrix<float, 2> J_func(Vector2f x, NfParams p);
	Vector2f func(Vector2f x, NfParams p);

	void parameters_update();

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	hrt_abstime _last_run{0};
	hrt_abstime _timestamp_sample{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HY_AIRAPEED_TRIM>) _param_hy_airspeed_trim,
		(ParamFloat<px4::params::HY_ALPHA_TRIM>) _param_hy_alpha_trim,
		(ParamInt<px4::params::HY_SPEED_SELECT>) _param_hy_speed_select,
		(ParamFloat<px4::params::HY_MAX_THRUST>) _param_hy_max_thrust,
		(ParamFloat<px4::params::HY_WING_R_CL>) _param_hy_wing_r_cl,
		(ParamFloat<px4::params::HY_WING_R_CL0>) _param_hy_wing_r_cl0,
		(ParamFloat<px4::params::HY_WING_R_CD>) _param_hy_wing_r_cd,
		(ParamFloat<px4::params::HY_WING_R_CD0>) _param_hy_wing_r_cd0,
		(ParamFloat<px4::params::HY_WING_R_AREA>) _param_hy_wing_r_area,
		(ParamFloat<px4::params::HY_HTAIL_CL>) _param_hy_htail_cl,
		(ParamFloat<px4::params::HY_HTAIL_CL0>) _param_hy_htail_cl0,
		(ParamFloat<px4::params::HY_HTAIL_CD>) _param_hy_htail_cd,
		(ParamFloat<px4::params::HY_HTAIL_CD0>) _param_hy_htail_cd0,
		(ParamFloat<px4::params::HY_HTAIL_AREA>) _param_hy_htail_area,
		(ParamFloat<px4::params::HY_WING_ANG_MAX>) _param_hy_wing_ang_max

	)

};
