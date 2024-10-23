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
#include <lib/npfg/npfg.hpp>
#include <lib/tecs/TECS.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
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
#include <uORB/topics/flight_phase_estimation.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/launch_detection_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_landing_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/orbit_status.h>
#include <uORB/uORB.h>


using namespace time_literals;

using matrix::Vector2d;
using matrix::Vector2f;

// [m] initial distance of waypoint in front of plane in heading hold mode
static constexpr float HDG_HOLD_DIST_NEXT = 3000.0f;

// [m] distance (plane to waypoint in front) at which waypoints are reset in heading hold mode
static constexpr float HDG_HOLD_REACHED_DIST = 1000.0f;

// [m] distance by which previous waypoint is set behind the plane
static constexpr float HDG_HOLD_SET_BACK_DIST = 100.0f;

// [rad/s] max yawrate at which plane locks yaw for heading hold mode
static constexpr float HDG_HOLD_YAWRATE_THRESH = 0.15f;

// [.] max manual roll/yaw normalized input from user which does not change the locked heading
static constexpr float HDG_HOLD_MAN_INPUT_THRESH = 0.01f;

// [us] time after which we abort landing if terrain estimate is not valid. this timer start whenever the terrain altitude
// was previously valid, and has changed to invalid.
static constexpr hrt_abstime TERRAIN_ALT_TIMEOUT = 1_s;

// [us] within this timeout, if a distance sensor measurement not yet made, the land waypoint altitude is used for terrain
// altitude. this timer starts at the beginning of the landing glide slope.
static constexpr hrt_abstime TERRAIN_ALT_FIRST_MEASUREMENT_TIMEOUT = 10_s;

// [.] max throttle from user which will not lead to motors spinning up in altitude controlled modes
static constexpr float THROTTLE_THRESH = -.9f;

// [m/s/s] slew rate limit for airspeed setpoint changes
static constexpr float ASPD_SP_SLEW_RATE = 1.f;

// [us] time after which the wind estimate is disabled if no longer updating
static constexpr hrt_abstime WIND_EST_TIMEOUT = 10_s;

// [s] minimum time step between auto control updates
static constexpr float MIN_AUTO_TIMESTEP = 0.01f;

// [s] maximum time step between auto control updates
static constexpr float MAX_AUTO_TIMESTEP = 0.05f;

// [rad] minimum pitch while airspeed has not yet reached a controllable value in manual position controlled takeoff modes
static constexpr float MIN_PITCH_DURING_MANUAL_TAKEOFF = 0.0f;

// [m] arbitrary buffer altitude added to clearance altitude setpoint during takeoff to ensure aircraft passes the clearance
// altitude while waiting for navigator to flag it exceeded
static constexpr float kClearanceAltitudeBuffer = 10.0f;

// [m/s] maximum rate at which the touchdown position can be nudged
static constexpr float MAX_TOUCHDOWN_POSITION_NUDGE_RATE = 4.0f;

// [.] normalized deadzone threshold for manual nudging input
static constexpr float MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE = 0.15f;

// [s] time interval after touchdown for ramping in runway clamping constraints (touchdown is assumed at FW_LND_TD_TIME after start of flare)
static constexpr float POST_TOUCHDOWN_CLAMP_TIME = 0.5f;

// [m/s] maximum reference altitude rate threshhold
static constexpr float MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT = 0.1f;

// [s] Timeout that has to pass in roll-constraining failsafe before warning is triggered
static constexpr uint64_t ROLL_WARNING_TIMEOUT = 2_s;

// [-] Can-run threshold needed to trigger the roll-constraining failsafe warning
static constexpr float ROLL_WARNING_CAN_RUN_THRESHOLD = 0.9f;


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

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};

	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_sp_pub;
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};

	uORB::Publication<position_controller_status_s>	_pos_ctrl_status_pub{ORB_ID(position_controller_status)};


	manual_control_setpoint_s _manual_control_setpoint{};
	position_setpoint_triplet_s _pos_sp_triplet{};
	vehicle_attitude_setpoint_s _att_sp{};
	vehicle_control_mode_s _control_mode{};
	vehicle_local_position_s _local_pos{};
	vehicle_status_s _vehicle_status{};


	bool _position_setpoint_previous_valid{false};
	bool _position_setpoint_current_valid{false};
	bool _position_setpoint_next_valid{false};

	perf_counter_t _loop_perf; // loop performance counter

	// [us] Last absolute time position control has been called
	hrt_abstime _last_time_position_control_called{0};

	uint8_t _position_sp_type{0};

	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_AUTO_ALTITUDE,
		FW_POSCTRL_MODE_AUTO_CLIMBRATE,
		FW_POSCTRL_MODE_AUTO_TAKEOFF,
		FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT,
		FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR,
		FW_POSCTRL_MODE_AUTO_PATH,
		FW_POSCTRL_MODE_MANUAL_POSITION,
		FW_POSCTRL_MODE_MANUAL_ALTITUDE,
		FW_POSCTRL_MODE_TRANSITON,
		FW_POSCTRL_MODE_OTHER
	} _control_mode_current{FW_POSCTRL_MODE_OTHER}; // used to check if the mode has changed

	// VEHICLE STATES

	double _current_latitude{0};
	double _current_longitude{0};
	float _current_altitude{0.f};

	float _roll{0.f};
	float _pitch{0.0f};
	float _yaw{0.0f};
	float _yawrate{0.0f};

	float _body_acceleration_x{0.f};
	float _body_velocity_x{0.f};

	// MANUAL MODES

	// indicates whether we have completed a manual takeoff in a position control mode
	bool _completed_manual_takeoff{false};

	// [rad] yaw setpoint for manual position mode heading hold
	float _hdg_hold_yaw{0.0f};

	bool _hdg_hold_enabled{false}; // heading hold enabled
	bool _yaw_lock_engaged{false}; // yaw is locked for heading hold

	position_setpoint_s _hdg_hold_position{}; // position where heading hold started

	// [.] normalized setpoint for manual altitude control [-1,1]; -1,0,1 maps to min,zero,max height rate commands
	float _manual_control_setpoint_for_height_rate{0.0f};

	// [.] normalized setpoint for manual airspeed control [-1,1]; -1,0,1 maps to min,cruise,max airspeed commands
	float _manual_control_setpoint_for_airspeed{0.0f};

	// [m/s] airspeed setpoint for manual modes commanded via MAV_CMD_DO_CHANGE_SPEED
	float _commanded_manual_airspeed_setpoint{NAN};



	// AIRSPEED

	float _airspeed_eas{0.f};
	float _eas2tas{1.f};
	bool _airspeed_valid{false};
	float _water_density = 1000;

	// Update our local parameter cache.
	void parameters_update();

	// Update subscriptions
	void airspeed_poll();
	void control_update();
	void manual_control_setpoint_poll();
	void vehicle_attitude_poll();
	void vehicle_command_poll();
	void vehicle_control_mode_poll();
	void vehicle_status_poll();

	void status_publish();

	void publishLocalPositionSetpoint(const position_setpoint_s &current_waypoint);


	/**
	 * @brief Vehicle control for position waypoints.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_position(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				   const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);


	/**
	 * @brief Vehicle control for following a path.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_path(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
			       const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Controls a desired airspeed, bearing, and height rate.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_velocity(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				   const position_setpoint_s &pos_sp_curr);


	/* manual control methods */

	/**
	 * @brief Controls altitude and airspeed, user commands roll setpoint.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 */
	void control_manual_altitude(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed);

	/**
	 * @brief Controls user commanded altitude, airspeed, and bearing.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 */
	void control_manual_position(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed);


	float get_manual_airspeed_setpoint();


	/**
	 * @brief Constrains the roll angle setpoint near ground to avoid wingtip strike.
	 *
	 * @param roll_setpoint Unconstrained roll angle setpoint [rad]
	 * @param altitude Vehicle altitude (AMSL) [m]
	 * @param terrain_altitude Terrain altitude (AMSL) [m]
	 * @return Constrained roll angle setpoint [rad]
	 */
	float constrainRollNearGround(const float roll_setpoint, const float altitude, const float terrain_altitude) const;


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_GND_SPD_MIN>) _param_fw_gnd_spd_min,


	)

};

#endif // HYDROPOSITIONCONTROL_HPP_
