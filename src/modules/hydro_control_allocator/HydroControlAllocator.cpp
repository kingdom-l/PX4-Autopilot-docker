/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file HydroControlAllocator.cpp
 *
 * Hydro Control allocator.
 *
 *
 */
#include "HydroControlAllocator.hpp"


HydroControlAllocator::HydroControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_hydro_motors_pub.advertise();
	_hydro_servos_pub.advertise();
	parameters_update();
}

HydroControlAllocator::~HydroControlAllocator()
{
	perf_free(_loop_perf);
}

bool
HydroAllocator::init()
{
	if (!_hydro_torque_setpoint_sub.registerCallback() || !_hydro_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
HydroAllocator::parameters_update()
{
	_st_info.x2 = 0;
	_st_info.y2 = 0;
	_st_info.z2 = 0.2;
	_st_info.yT = 0.1;
	_st_info.yh = 0.1;
	_st_info.xe = 0.6;
	_hy_effectiveness = {{        1,                          0,                       1,                      0,                 1,        0},
			    {         0,                         -1,                       0,                     -1,                 0,        1},
			    {         0,                 -(_st_info.y2+_st_info.yT),       0,             -(_st_info.y2-_st_info.yT), 0,        0},
			    {  _st_info.z2,                _st_info.x2,             _st_info.z2,             _st_info.x2,             0, _st_info.xe},
			    {-(_st_info.y2+_st_info.yT),           0,        -(_st_info.y2-_st_info.yT),            0,                0,         0}};
	matrix::geninv(_hy_effectiveness, _hy_mix);

}

void HydroControlAllocator::optim(float x_opt[2], NfParams p)
{

}

SquareMatrix<float, 2> HydroControlAllocator::J_func(Vector2f x, NfParams p)
{

}

Vector2f HydroControlAllocator::func(Vector2f x, NfParams p)
{

}

void HydroControlAllocator::Run()
{
	if (should_exit()) {
		_hydro_torque_setpoint_sub.unregisterCallback();
		_hydro_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	bool do_update = false;
	vehicle_torque_setpoint_s hydro_torque_setpoint;
	vehicle_thrust_setpoint_s hydro_thrust_setpoint;
	// Run allocator on torque changes
	if (_hydro_torque_setpoint_sub.update(&hydro_torque_setpoint)) {
		_wrench_sp(2) = hydro_torque_setpoint.xyz[0];
		_wrench_sp(3) = hydro_torque_setpoint.xyz[1];
		_wrench_sp(4) = hydro_torque_setpoint.xyz[2];
		// PX4_INFO("_torque_sp: %f %f %f", (double)_torque_sp(0), (double)_torque_sp(1), (double)_torque_sp(2));
		do_update = true;
		_timestamp_sample = hydro_torque_setpoint.timestamp_sample;
	}

	// Also run allocator on thrust setpoint changes if the torque setpoint
	// has not been updated for more than 5ms
	if (_vehicle_thrust_setpoint_sub.update(&hydro_thrust_setpoint)) {
		_wrench_sp(0) = hydro_thrust_setpoint.xyz[0];
		_wrench_sp(1) = hydro_thrust_setpoint.xyz[2];

		if (dt > 0.005f) {
			do_update = true;
			_timestamp_sample = hydro_thrust_setpoint.timestamp_sample;
		}
	}

	if(do_update){
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);
		Matrix::Vector<float, 6> force_sp = _hy_mix * _wrench_sp;


	}

	perf_end(_loop_perf);
}

int HydroControlAllocator::task_spawn(int argc, char *argv[])
{
	HydroControlAllocator *instance = new HydroControlAllocator();

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

int HydroControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HydroControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
HydroControlAllocator.

)DESCR_STR");

	return 0;
}

extern "C" __EXPORT int hydro_control_allocator_main(int argc, char *argv[])
{
	return HydroControlAllocator::main(argc, argv);
}
