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
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/uORB.h>
#include <poll.h>

#include <lib/two_order_eso/two_order_eso.hpp>
#include <lib/three_order_eso/three_order_eso.hpp>
#include <lib/tracking_differentiator/tracking_differentiator.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#include "HydroAdvancedControllers.hpp"


// #include <lib/Eigen/Eigen.h>

using namespace time_literals;

using matrix::Vector2d;
using matrix::Vector2f;

typedef struct
{
	float vel;           // 速度
	float pos[100];      // 当前位置
	uint32_t index;      // 索引
	float temp_res; // 便于计算的中间量
	float temp_time_sum; // 便于计算的中间量
	float temp_res_sub[50]; // 便于计算的中间量
	hrt_abstime last_time; // 上一次时间
	// 用于计算dt，从而计算速度
	uint8_t init_flag; // 标记是否初始化
	float dt[100];
} time_derivative_t;

/** State of one independently filtered motion-capture position axis. */
struct AxisJumpFilterState {
	float value{0.f};
	float candidate{0.f};
	hrt_abstime last_accept_time{0};
	uint8_t reject_count{0};
	uint8_t candidate_count{0};
	bool initialized{false};
};

enum class AxisFilterResult : uint8_t {
	Rejected = 0,
	Accepted,
	Reacquired
};

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

	uint8_t TimeDerivativeCalc(uint8_t times, time_derivative_t *ins, float position);


private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _debug_vect_sub{ORB_ID(debug_vect)}; // 订阅动捕位置
	// uORB::Subscription _debug_sub{ORB_ID(debug_key_value)}; // 订阅动捕高度
	uORB::Subscription _depth_estimated_sub{ORB_ID(depth_estimated)}; // depth gauge
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<vehicle_attitude_setpoint_s>	_hy_att_sp_pub{ORB_ID(hy_vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _vehicle_local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<trajectory_setpoint_s> _hy_traj_sp_pub{ORB_ID(trajectory_setpoint)};
	struct debug_value_s _dbg_val;
	orb_advert_t pub_dbg_val;

	struct debug_vect_s _debug_vec{}; // 通过逐轴跳点检查后的动捕位置
	AxisJumpFilterState _debug_x_filter{};
	AxisJumpFilterState _debug_y_filter{};
	AxisJumpFilterState _debug_z_filter{};
	struct debug_vect_s _eadrc_raw_debug_vec{}; // eADRC-HRP专用：不经过跳点检测
	hrt_abstime _eadrc_raw_last_sample_time{0};
	bool _eadrc_raw_feedback_initialized{false};

	struct debug_array_s _dbg_arr;
	orb_advert_t pub_dbg_arr;

	vehicle_local_position_setpoint_s _pos_sp{};

	trajectory_setpoint_s _traj_sp{};
	depth_estimated_s _depth_estimated{};
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_status_s _vehicle_status{};


	perf_counter_t _loop_perf; // loop performance counter
	hrt_abstime _last_run{0};
	matrix::Dcmf _R{matrix::eye<float, 3>()};

	TwoOrderEso _vel_eso{2.3f, 100, 300, 0.008};
	ThreeOrderEso _depth_eso{2.3f, 100, 300, 1000, 0.008};
	// TrackingDifferentiator _pos_x_td{0.01, 100, 0.07};
	// TrackingDifferentiator _pos_y_td{0.01, 100, 0.07};
	// TrackingDifferentiator _pos_z_td{0.01, 100, 0.07};
	math::LowPassFilter2p<float> _pos_x_lpf{800.f, 40.f}, _pos_z_lpf{800.f, 40.f};
	float _vx_hat = 0.f, _px_hat = 0.f, _vy_hat = 0.f, _py_hat = 0.f, _vz_hat = 0.f, _pz_hat = 0.f;
	float _Va_hat = 0.f;
	// Eigen::MatrixXf _mat(3, 3);


	float _water_density = 1000;
	float _depth_e = 0.f, _depth_e_i = 0.f;
	float _depth_e_pre = 0.f;
	float _Va_e = 0.f, _Va_e_pre = 0.f, _Va_e_i = 0.f;
	float _fx_sp = 0., _fz_sp = 0.;

	hrt_abstime _time_now{0};
	time_derivative_t _posx_derivate = {0}, _posy_derivate = {0}, _posz_derivate = {0};
	bool _vx_derivative_ready{false};
	bool _vy_derivative_ready{false};
	bool _vz_derivative_ready{false};
	bool _derivative_ready{false};
	time_derivative_t _eadrc_posx_derivate = {0}, _eadrc_posy_derivate = {0}, _eadrc_posz_derivate = {0};
	bool _eadrc_vx_derivative_ready{false};
	bool _eadrc_vy_derivative_ready{false};
	bool _eadrc_vz_derivative_ready{false};

	static constexpr uint8_t JumpTimeoutReacquireSamples = 2;
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
	float mapForwardForceToThrottle(float force, float resolution, float force_scale) const;
	float mapPhysicalForwardForceToThrottle(float force, float resolution, float maximum_force) const;
	void resetControllerStates(int controller_mode, float depth_error, float depth_error_rate, float velocity_error);
	void resetSactStates(float velocity_error);
	AxisFilterResult filterPositionAxis(float raw_value, float jump_threshold,
			hrt_abstime now, uint8_t reacquire_samples, hrt_abstime reacquire_timeout_us,
			AxisJumpFilterState &state);
	void resetTimeDerivative(time_derivative_t &state, float position, hrt_abstime now);
	void updateAxisDerivative(AxisFilterResult filter_result, const AxisJumpFilterState &filter_state,
			float position, hrt_abstime now, uint8_t derivative_window,
			hrt_abstime stale_timeout_us, time_derivative_t &derivative_state, bool &derivative_ready);
	bool axisMeasurementFresh(const AxisJumpFilterState &state, hrt_abstime now,
			hrt_abstime timeout_us) const;
	float slewTowards(float current, float target, float time_constant, float dt) const;

	static constexpr int ControllerAdrc = 0;
	static constexpr int ControllerPid = 1;
	static constexpr int ControllerEadrcHrp = 2;
	static constexpr int ControllerSactPlus = 3;

	EadrcHrpController _eadrc_hrp{};
	hrt_abstime _eadrc_update_timestamp{0};
	// Keep the continuously evaluated PD/feedforward path separate from the two
	// adaptive channels. SactPlusController::update(..., false) clears its own
	// adaptive states, so a dedicated base instance is required to freeze the
	// depth and velocity Lambda/Theta states independently during re-warmup.
	SactPlusController _sact_base{};
	SactPlusController _sact_depth_adaptive{};
	SactPlusController _sact_velocity_adaptive{};
	int _controller_mode_previous{-1};
	bool _pid_active{false};
	bool _eadrc_active{false};
	bool _sact_active{false};
	bool _sact_depth_memory_valid{false};
	bool _sact_velocity_memory_valid{false};
	bool _sact_depth_frozen{false};
	bool _sact_velocity_frozen{false};
	bool _sact_depth_recovering{false};
	bool _sact_velocity_recovering{false};
	float _sact_depth_compensation_hold{0.f};
	float _sact_velocity_compensation_hold{0.f};
	float _sact_depth_recovery_start{0.f};
	float _sact_velocity_recovery_start{0.f};
	float _sact_depth_recovery_elapsed{0.f};
	float _sact_velocity_recovery_elapsed{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HY_DEP_P>) _param_hy_dep_p,
		(ParamFloat<px4::params::HY_DEP_I>) _param_hy_dep_i,
		(ParamFloat<px4::params::HY_DEP_FF>) _param_hy_dep_ff,
		(ParamFloat<px4::params::HY_DEP_LIM>) _param_hy_dep_lim,
		(ParamFloat<px4::params::HY_D_PMAX>) _param_hy_d_pmax,
		(ParamFloat<px4::params::HY_D_RMAX>) _param_hy_d_rmax,
		(ParamFloat<px4::params::HY_DEPSAT_MAX>) _param_hy_depsat_max,
		(ParamFloat<px4::params::HY_DEPSAT_K>) _param_hy_depsat_k,
		(ParamFloat<px4::params::HY_DEPTH_SP>) _param_hy_depth_sp,
		(ParamFloat<px4::params::HY_VELFB_P>) _param_hy_velfb_p,
		(ParamFloat<px4::params::HY_VA_P>) _param_hy_va_p,
		(ParamFloat<px4::params::HY_VA_I>) _param_hy_va_i,
		(ParamFloat<px4::params::HY_VA_FF>) _param_hy_va_ff,
		(ParamFloat<px4::params::HY_VA_SP>) _param_hy_va_sp,
		(ParamFloat<px4::params::HY_VE_RES>) _param_hy_ve_res,
		(ParamFloat<px4::params::HY_VFX_SP_SLOPE>) _param_hy_vfx_sp_slope,
		(ParamFloat<px4::params::HY_VE_A>) _param_hy_ve_a,
		(ParamFloat<px4::params::HY_VE_B>) _param_hy_ve_b,
		(ParamFloat<px4::params::HY_VE_ILIMIT>) _param_hy_ve_ilimit,
		(ParamFloat<px4::params::HY_DE_A>) _param_hy_de_a,
		(ParamFloat<px4::params::HY_DE_B>) _param_hy_de_b,
		(ParamFloat<px4::params::HY_DE_ILIMIT>) _param_hy_de_ilimit,

		(ParamFloat<px4::params::HY_POS_TD_H>) _param_hy_pos_td_h,
		(ParamFloat<px4::params::HY_POS_TD_R0>) _param_hy_pos_td_r0,
		(ParamFloat<px4::params::HY_POS_TD_H0>) _param_hy_pos_td_h0,
		(ParamFloat<px4::params::HY_DBG_JUMP>) _param_hy_dbg_jump,
		(ParamInt<px4::params::HY_VEL_WIN>) _param_hy_vel_win,
		(ParamInt<px4::params::HY_JMP_REJ_N>) _param_hy_jump_reject_count,
		(ParamFloat<px4::params::HY_JMP_REAC_T>) _param_hy_jump_reacquire_time,
		(ParamFloat<px4::params::HY_POS_TIMEOUT>) _param_hy_position_timeout,
		(ParamFloat<px4::params::HY_FB_RAMP>) _param_hy_feedback_ramp,
		(ParamFloat<px4::params::HY_DEP_SAMFREQ>) _param_hy_dep_samfreq,
		(ParamFloat<px4::params::HY_DEP_CUTFREQ>) _param_hy_dep_cutfreq,
		(ParamFloat<px4::params::HY_D_ESO_BETA1>) _param_hy_d_eso_beta1,
		(ParamFloat<px4::params::HY_D_ESO_BETA2>) _param_hy_d_eso_beta2,
		(ParamFloat<px4::params::HY_D_ESO_BETA3>) _param_hy_d_eso_beta3,
		(ParamFloat<px4::params::HY_D_ESO_B0_INV>) _param_hy_d_eso_b0_inv,
		(ParamFloat<px4::params::HY_DEP_ADRC_P>) _param_hy_dep_adrc_p,
		(ParamFloat<px4::params::HY_DEP_ADRC_D>) _param_hy_dep_adrc_d,
		(ParamFloat<px4::params::HY_DEP_FF_ADRC>) _param_hy_dep_ff_adrc,
		(ParamFloat<px4::params::HY_DEP_KCOMP_ESO>) _param_hy_dep_kcomp_eso,
		(ParamFloat<px4::params::HY_DEP_LIM_ADRC>) _param_hy_dep_lim_adrc,
		(ParamFloat<px4::params::HY_V_ESO_BETA1>) _param_hy_v_eso_beta1,
		(ParamFloat<px4::params::HY_V_ESO_BETA2>) _param_hy_v_eso_beta2,
		(ParamFloat<px4::params::HY_V_ESO_B0_INV>) _param_hy_v_eso_b0_inv,
		(ParamFloat<px4::params::HY_VA_ADRC_P>) _param_hy_va_adrc_p,
		(ParamFloat<px4::params::HY_VA_FF_ADRC>) _param_hy_va_ff_adrc,
		(ParamFloat<px4::params::HY_VA_ADRC_LIM>) _param_hy_va_adrc_lim,
		(ParamFloat<px4::params::HY_VE_RES_ADRC>) _param_hy_ve_res_adrc,
		(ParamFloat<px4::params::HY_VFX_SLPADRC>) _param_hy_vfx_sp_slpadrc,
		(ParamInt<px4::params::HY_DEPVA_PID_EN>) _param_hy_depva_pid_en,
		(ParamFloat<px4::params::HY_D_ESO_H>) _param_hy_d_eso_h,
		(ParamFloat<px4::params::HY_V_ESO_H>) _param_hy_v_eso_h,
		(ParamFloat<px4::params::HY_THRUST_MAX>) _param_hy_thrust_max,

		(ParamFloat<px4::params::HY_HR_D_KP>) _param_hy_hr_d_kp,
		(ParamFloat<px4::params::HY_HR_D_KD>) _param_hy_hr_d_kd,
		(ParamFloat<px4::params::HY_HR_DEP_FF>) _param_hy_hr_dep_ff,
		(ParamFloat<px4::params::HY_HR_D_B0_INV>) _param_hy_hr_d_b0_inv,
		(ParamFloat<px4::params::HY_HR_D_WO>) _param_hy_hr_d_wo,
		(ParamFloat<px4::params::HY_HR_D_Z3_INIT>) _param_hy_hr_d_z3_init,
		(ParamFloat<px4::params::HY_HR_V_KP>) _param_hy_hr_v_kp,
		(ParamFloat<px4::params::HY_HR_VA_FF>) _param_hy_hr_va_ff,
		(ParamFloat<px4::params::HY_HR_V_B0_INV>) _param_hy_hr_v_b0_inv,
		(ParamFloat<px4::params::HY_HR_V_WO>) _param_hy_hr_v_wo,
		(ParamFloat<px4::params::HY_HR_D_ALP>) _param_hy_hr_d_alp,
		(ParamFloat<px4::params::HY_HR_V_ALP>) _param_hy_hr_v_alp,
		(ParamFloat<px4::params::HY_HR_FORGET>) _param_hy_hr_forget,
		(ParamFloat<px4::params::HY_HR_RIDGE>) _param_hy_hr_ridge,
		(ParamFloat<px4::params::HY_HR_RLPF>) _param_hy_hr_rlpf,
		(ParamFloat<px4::params::HY_HR_D_RLIM>) _param_hy_hr_d_rlim,
		(ParamFloat<px4::params::HY_HR_V_RLIM>) _param_hy_hr_v_rlim,
		(ParamFloat<px4::params::HY_HR_D_SIG>) _param_hy_hr_d_sig,
		(ParamFloat<px4::params::HY_HR_V_SIG>) _param_hy_hr_v_sig,
		(ParamFloat<px4::params::HY_HR_C_TC>) _param_hy_hr_c_tc,
		(ParamFloat<px4::params::HY_HR_RAMP>) _param_hy_hr_ramp,

		(ParamFloat<px4::params::HY_SA_D_SP>) _param_hy_sa_d_sp,
		(ParamFloat<px4::params::HY_SA_D_SD>) _param_hy_sa_d_sd,
		(ParamFloat<px4::params::HY_SA_D_DP>) _param_hy_sa_d_dp,
		(ParamFloat<px4::params::HY_SA_D_DD>) _param_hy_sa_d_dd,
		(ParamFloat<px4::params::HY_SA_DEP_FF>) _param_hy_sa_dep_ff,
		(ParamFloat<px4::params::HY_SA_D_B0_INV>) _param_hy_sa_d_b0_inv,
		(ParamFloat<px4::params::HY_SA_D_DLP>) _param_hy_sa_d_dlp,
		(ParamFloat<px4::params::HY_SA_D_DLD>) _param_hy_sa_d_dld,
		(ParamFloat<px4::params::HY_SA_D_A1>) _param_hy_sa_d_a1,
		(ParamFloat<px4::params::HY_SA_D_A2>) _param_hy_sa_d_a2,
		(ParamFloat<px4::params::HY_SA_D_GAM>) _param_hy_sa_d_gam,
		(ParamFloat<px4::params::HY_SA_D_DNM>) _param_hy_sa_d_dnm,
		(ParamFloat<px4::params::HY_SA_D_TLM>) _param_hy_sa_d_tlm,
		(ParamFloat<px4::params::HY_SA_D_LLM>) _param_hy_sa_d_llm,
		(ParamFloat<px4::params::HY_SA_V_SP>) _param_hy_sa_v_sp,
		(ParamFloat<px4::params::HY_SA_V_SD>) _param_hy_sa_v_sd,
		(ParamFloat<px4::params::HY_SA_V_DP>) _param_hy_sa_v_dp,
		(ParamFloat<px4::params::HY_SA_V_DD>) _param_hy_sa_v_dd,
		(ParamFloat<px4::params::HY_SA_VA_FF>) _param_hy_sa_va_ff,
		(ParamFloat<px4::params::HY_SA_V_B0_INV>) _param_hy_sa_v_b0_inv,
		(ParamFloat<px4::params::HY_SA_V_DLP>) _param_hy_sa_v_dlp,
		(ParamFloat<px4::params::HY_SA_V_DLD>) _param_hy_sa_v_dld,
		(ParamFloat<px4::params::HY_SA_V_A1>) _param_hy_sa_v_a1,
		(ParamFloat<px4::params::HY_SA_V_A2>) _param_hy_sa_v_a2,
		(ParamFloat<px4::params::HY_SA_V_GAM>) _param_hy_sa_v_gam,
		(ParamFloat<px4::params::HY_SA_V_DNM>) _param_hy_sa_v_dnm,
		(ParamFloat<px4::params::HY_SA_V_TLM>) _param_hy_sa_v_tlm,
		(ParamFloat<px4::params::HY_SA_V_LLM>) _param_hy_sa_v_llm,
		(ParamFloat<px4::params::HY_SA_DER_TC>) _param_hy_sa_der_tc,
		(ParamFloat<px4::params::HY_SA_RAMP>) _param_hy_sa_ramp
	)

};

#endif // HYDROPOSITIONCONTROL_HPP_
