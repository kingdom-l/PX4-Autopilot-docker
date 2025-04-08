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


/**
 * @file hy_pos_control_main.hpp
 * Implementation of various fixed-wing position level navigation/control modes.
 *
 * The implementation for the controllers is in a separate library. This class only
 * interfaces to the library.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#ifndef HYDROPOSITIONCONTROL_HPP_
#define HYDROPOSITIONCONTROL_HPP_

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/atmosphere/atmosphere.h>

#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/depth_estimated.h>
#include <uORB/uORB.h>
#include <poll.h>

#include <lib/two_order_eso/two_order_eso.hpp>
#include <lib/three_order_eso/three_order_eso.hpp>
#include <lib/tracking_differentiator/tracking_differentiator.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <uORB/topics/vehicle_local_position_setpoint.h>
// #include <lib/Eigen/Eigen.h>

using namespace time_literals;

using matrix::Vector2d;
using matrix::Vector2f;

class HydroPositionControl final : public ModuleBase<HydroPositionControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	HydroPositionControl();
	~HydroPositionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();


private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _debug_vect_sub{ORB_ID(debug_vect)}; // 订阅动捕位置
	// uORB::Subscription _debug_sub{ORB_ID(debug_key_value)}; // 订阅动捕高度
	uORB::Subscription _depth_estimated_sub{ORB_ID(depth_estimated)}; // depth gauge
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_sp_pub;
	uORB::Publication<vehicle_local_position_setpoint_s> _vehicle_local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	struct debug_value_s _dbg_val;
	orb_advert_t pub_dbg_val;

	struct debug_array_s _dbg_arr;
	orb_advert_t pub_dbg_arr;

	vehicle_local_position_setpoint_s _pos_sp{};

	vehicle_local_position_s _local_pos{};
	depth_estimated_s _depth_estimated{};
	manual_control_setpoint_s _manual_control_setpoint{};

	perf_counter_t _loop_perf; // loop performance counter
	hrt_abstime _last_run{0};
	matrix::Dcmf _R{matrix::eye<float, 3>()};

	TwoOrderEso _depth_eso{5.0f, 100, 300};
	// ThreeOrderEso _depth_eso1{100, 300, 1000};
	TrackingDifferentiator _pos_x_td{0.01, 100, 0.07};
	math::LowPassFilter2p<float> _pos_x_lpf{800.f, 40.f};
	float _vx_hat, _px_hat;
	// Eigen::MatrixXf _mat(3, 3);

	float _water_density = 1000;
	float _depth_e_pre = 0.f;
	float _depth_e_i = 0.f;

	/**
	 * @brief Constrains the roll angle setpoint near ground to avoid wingtip strike.
	 *
	 * @param roll_setpoint Unconstrained roll angle setpoint [rad]
	 * @param altitude Vehicle altitude (AMSL) [m]
	 * @param terrain_altitude Terrain altitude (AMSL) [m]
	 * @return Constrained roll angle setpoint [rad]
	 */
	// float constrainRollNearGround(const float roll_setpoint, const float altitude, const float terrain_altitude) const;

	// int parameters_update();

	float saturate_function(float x, float max_value, float k, float center);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HY_DEP_P>) _param_hy_dep_p,
		(ParamFloat<px4::params::HY_DEP_I>) _param_hy_dep_i,
		(ParamFloat<px4::params::HY_DEP_FF>) _param_hy_dep_ff,
		(ParamFloat<px4::params::HY_DEP_LIM>) _param_hy_dep_lim,
		(ParamFloat<px4::params::HY_P_LIM>) _param_hy_p_lim,
		(ParamFloat<px4::params::HY_R_LIM>) _param_hy_r_lim,
		(ParamFloat<px4::params::HY_DEPSAT_MAX>) _param_hy_depsat_max,
		(ParamFloat<px4::params::HY_DEPSAT_K>) _param_hy_depsat_k,
		(ParamFloat<px4::params::HY_DEPTH_SP>) _param_hy_depth_sp,
		(ParamFloat<px4::params::HY_POS_TD_H>) _param_hy_pos_td_h,
		(ParamFloat<px4::params::HY_POS_TD_R0>) _param_hy_pos_td_r0,
		(ParamFloat<px4::params::HY_POS_TD_H0>) _param_hy_pos_td_h0,
		(ParamFloat<px4::params::HY_D_ESO_BETA1>) _param_hy_d_eso_beta1,
		(ParamFloat<px4::params::HY_D_ESO_BETA2>) _param_hy_d_eso_beta2,
		(ParamFloat<px4::params::HY_D_ESO_B0>) _param_hy_d_eso_b0

	)

};

#endif // HYDROPOSITIONCONTROL_HPP_
