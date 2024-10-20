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
	struct StructureInfo{
		float x2;
		float y2;
		float z2;
		float yT;
		float yh;
		float xe;
	};
	StructureInfo _structure_info;
	matrix::Matrix<float, 5, 6> effectiveness;

	struct NfParams{
		float Cl;
		float Cl0;
		float Cd;
		float Cd0;
		float S_wing;
		float Fx;
		float Fz;
	};
	NfParams _nf_params;

	void optim(float x_opt[2], NfParams p);
	SquareMatrix<float, 2> J_func(Vector2f x, NfParams p);
	Vector2f func(Vector2f x, NfParams p);

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	ParamHandles _param_handles{};
	Params _params{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HY_AIRAPEED_TRIM>) _param_hy_airspeed_trim,
		(ParamInt<px4::params::HY_SPEED_SELECT>) _param_hy_speed_select,
		(ParamFloat<px4::params::HY_MAX_THRUST>) _param_hy_max_thrust,
		(ParamFloat<px4::params::HY_WING_RCL>) _param_hy_wing_r_cl,
		(ParamFloat<px4::params::HY_WING_RCL0>) _param_hy_wing_r_cl0,
		(ParamFloat<px4::params::HY_WING_RCD>) _param_hy_wing_r_cd,
		(ParamFloat<px4::params::HY_WING_RCD0>) _param_hy_wing_r_cd0,
		(ParamFloat<px4::params::HY_HTAIL_CL>) _param_hy_htail_cl,
		(ParamFloat<px4::params::HY_HTAIL_CL0>) _param_hy_htail_cl0,
		(ParamFloat<px4::params::HY_HTAIL_CD>) _param_hy_htail_cd,
		(ParamFloat<px4::params::HY_HTAIL_CD0>) _param_hy_htail_cd0,
		(ParamFloat<px4::params::HY_WING_ANG_MAX>) _param_hy_wing_ang_max,

	)

};
