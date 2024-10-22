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

using namespace matrix;

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
HydroControlAllocator::init()
{
	if (!_hydro_torque_setpoint_sub.registerCallback() || !_hydro_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
HydroControlAllocator::parameters_update()
{
	_st_info.x2 = 0;
	_st_info.y2 = 0;
	_st_info.z2 = 0.2;
	_st_info.yT = 0.1;
	_st_info.yh = 0.1;
	_st_info.xe = 0.6;

	_hy_effectiveness(0,0) = 1; _hy_effectiveness(0,1) = 0; _hy_effectiveness(0,2) = 1;
	_hy_effectiveness(0,3) = 0; _hy_effectiveness(0,4) = 1; _hy_effectiveness(0,5) = 0;

	_hy_effectiveness(1,0) = 0; _hy_effectiveness(1,1) = -1; _hy_effectiveness(1,2) = 0;
	_hy_effectiveness(1,3) = -1; _hy_effectiveness(1,4) = 0; _hy_effectiveness(1,5) = 1;

	_hy_effectiveness(2,0) = 0;                          _hy_effectiveness(2,1) = -(_st_info.y2+_st_info.yT); _hy_effectiveness(2,2) = 0;
	_hy_effectiveness(2,3) = -(_st_info.y2-_st_info.yT); _hy_effectiveness(2,4) = 0;                          _hy_effectiveness(2,5) = 0;

	_hy_effectiveness(3,0) = _st_info.z2; _hy_effectiveness(3,1) = _st_info.x2; _hy_effectiveness(3,2) = _st_info.z2;
	_hy_effectiveness(3,3) = _st_info.x2; _hy_effectiveness(3,4) = 0;           _hy_effectiveness(3,5) = _st_info.xe;

	_hy_effectiveness(4,0) = -(_st_info.y2+_st_info.yT); _hy_effectiveness(4,1) = 0; _hy_effectiveness(4,2) = -(_st_info.y2-_st_info.yT);
	_hy_effectiveness(4,3) = 0;                          _hy_effectiveness(4,4) = 0; _hy_effectiveness(4,5) = 0;


	// _hy_effectiveness = hy_effectiveness;
	matrix::geninv(_hy_effectiveness, _hy_mix);
	_nf_params_hy_wr.Cl = _param_hy_rhf_cl.get();
	_nf_params_hy_wr.Cl0 = _param_hy_rhf_cl0.get();
	_nf_params_hy_wr.Cd = _param_hy_rhf_cd.get();
	_nf_params_hy_wr.Cd0 = _param_hy_rhf_cd0.get();
	_nf_params_hy_wr.S_wing = _param_hy_rhf_area.get();
	_nf_params_hy_wr.with_thrust = true;

	_nf_params_hy_wl.Cl = _param_hy_rhf_cl.get();
	_nf_params_hy_wl.Cl0 = _param_hy_rhf_cl0.get();
	_nf_params_hy_wl.Cd = _param_hy_rhf_cd.get();
	_nf_params_hy_wl.Cd0 = _param_hy_rhf_cd0.get();
	_nf_params_hy_wl.S_wing = _param_hy_rhf_area.get();
	_nf_params_hy_wl.with_thrust = true;

	_nf_params_hy_htail.Cl = _param_hy_htail_cl.get();
	_nf_params_hy_htail.Cl0 = _param_hy_htail_cl0.get();
	_nf_params_hy_htail.Cd = _param_hy_htail_cd.get();
	_nf_params_hy_htail.Cd0 = _param_hy_htail_cd0.get();
	_nf_params_hy_htail.S_wing = _param_hy_htail_area.get();
	_nf_params_hy_htail.with_thrust = false;
}

SquareMatrix<float, 2> HydroControlAllocator::J_func(Vector2f x_opt, NfParams p)
{
	SquareMatrix<float, 2> J;
	if(p.with_thrust){
		float gamma = x_opt(0);
		float T = x_opt(1);

		J(0, 0) = T * sinf(gamma) - 0.5f * _rho * p.S_wing * _Va2 * p.Cl * sinf(_alpha) + 0.5f * _rho * p.S_wing * _Va2 * p.Cd * cosf(_alpha);
		J(0, 1) = -cosf(gamma);
		J(1, 0) =-T * cosf(gamma) - 0.5f * _rho * p.S_wing * _Va2 * p.Cl * cosf(_alpha) - 0.5f * _rho * p.S_wing * _Va2 * p.Cd * sinf(_alpha);
		J(1, 1) = -sinf(gamma);
	}else{
		J(0, 0) = - 0.5f * _rho * p.S_wing * _Va2 * p.Cl * sinf(_alpha) + 0.5f * _rho * p.S_wing * _Va2 * p.Cd * cosf(_alpha);
		J(0, 1) = 0;
		J(1, 0) = - 0.5f * _rho * p.S_wing * _Va2 * p.Cl * cosf(_alpha) - 0.5f * _rho * p.S_wing * _Va2 * p.Cd * sinf(_alpha);
		J(1, 1) = 0;
	}

	return J;
}

Vector2f HydroControlAllocator::func(Vector2f x_opt, NfParams p)
{
	float gamma = x_opt(0);
	float T = x_opt(1);
	Vector2f out;
	out(0) = p.Fx - T * cosf(gamma) - 0.5f * _rho * p.S_wing * _Va2 * (p.Cl*(gamma+_alpha)+p.Cl0) * sinf(_alpha) + 0.5f * _rho * p.S_wing * _Va2 * (p.Cd*(gamma+_alpha)+p.Cd0) * cosf(_alpha);
	out(1) = p.Fz - T * sinf(gamma) - 0.5f * _rho * p.S_wing * _Va2 * (p.Cl*(gamma+_alpha)+p.Cl0) * cosf(_alpha) - 0.5f * _rho * p.S_wing * _Va2 * (p.Cd*(gamma+_alpha)+p.Cd0) * sinf(_alpha);
	return out;
}

void HydroControlAllocator::optim(float x_opt[2], NfParams p)
{
	Vector2f x(x_opt);
	Vector2f func_out;
	SquareMatrix<float, 2> J;

	if(p.with_thrust){

		SquareMatrix<float, 2> J_inv;
		Vector2f delta_x;
		Vector2f x_new;

		for(int i = 0; i < 10; i++){
			J = J_func(x, p);
			func_out = func(x, p);
			inv(J, J_inv);

			delta_x = - J_inv * func_out;

			if(delta_x.norm_squared() < (float)1e-6)
				break;

			x_new = x + delta_x;

			x_new(0) = math::constrain(x_new(0), - _param_hy_wing_ang_max.get(), _param_hy_wing_ang_max.get());
			float x1_max = math::constrain(_param_hy_th_max_gain.get() * (_manual_control_setpoint.throttle+1)*0.5f, 0.f, 1.f) * _param_hy_thrust_max.get();
			x_new(1) = math::constrain(x_new(1), 0.f, x1_max);

			x = x_new;
		}
	}
	else{
		float delta_x;
		for(int i = 0; i < 10; i++){

			J = J_func(x, p);
			func_out = func(x, p);
			delta_x = -0.2f * 1.0f/J(0,0) * func_out(0) - 0.8f * 1.0f/J(1,0) * func_out(1);
			if(abs(delta_x) < (float)1e-6)
				break;

			x(0) = x(0) + delta_x;

			x(0) = math::constrain(x(0), - _param_hy_wing_ang_max.get(), _param_hy_wing_ang_max.get());
		}
	}
	x.copyTo(x_opt);
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

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

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
	if (_hydro_thrust_setpoint_sub.update(&hydro_thrust_setpoint)) {
		_wrench_sp(0) = hydro_thrust_setpoint.xyz[0];
		_wrench_sp(1) = hydro_thrust_setpoint.xyz[2];

		if (dt > 0.005f) {
			do_update = true;
			_timestamp_sample = hydro_thrust_setpoint.timestamp_sample;
		}
	}

	if(do_update){
		_last_run = now;

		if(_param_hy_speed_select.get() == 0)//使用替代速度
		{
			_Va2 = _param_hy_airspeed_trim.get() * _param_hy_airspeed_trim.get();
			_alpha = _param_hy_alpha_trim.get();
		}
		else
		{
			// 订阅空速信息计算Va2和攻角
			_Va2 = _param_hy_airspeed_trim.get(); //TODO 暂时不支持真实速度，都用替代速度
			_alpha = _param_hy_alpha_trim.get();
		}

		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);
		matrix::Vector<float, 6> force_sp = _hy_mix * _wrench_sp;

		_nf_params_hy_wr.Fx = force_sp(0);
		_nf_params_hy_wr.Fz = force_sp(1);
		_nf_params_hy_wl.Fx = force_sp(2);
		_nf_params_hy_wl.Fz = force_sp(3);
		_nf_params_hy_htail.Fx = force_sp(4);
		_nf_params_hy_htail.Fz = force_sp(5);

		float x_opt[3][2] = {{0, _nf_params_hy_wr.Fx/2},
			       {0, _nf_params_hy_wl.Fx/2},
			       {0, 0}}; // 初值取得可能有问题
		optim(x_opt[0], _nf_params_hy_wr);
		optim(x_opt[1], _nf_params_hy_wl);
		optim(x_opt[2], _nf_params_hy_htail); // 需要考虑如何融合计算得到的两个舵偏角以及舵机角度归一化

		printf("Here Hydro Control Allocator");

		//根据参数设置的对应关系填入数据并发送
		actuator_motors_s hydro_motors_msg{0};
		actuator_servos_s hydro_servos_msg{0};

		hydro_motors_msg.timestamp = hrt_absolute_time();
		hydro_motors_msg.timestamp_sample = hydro_thrust_setpoint.timestamp_sample;

		hydro_servos_msg.timestamp = hrt_absolute_time();
		hydro_servos_msg.timestamp_sample = hydro_torque_setpoint.timestamp_sample;

		// 此处hy_rt_idx[0]对应右边电机的Motor编号
		hydro_motors_msg.control[_param_hy_rmotor_idx.get() - 1] = x_opt[0][1]; // 右水翼电机
		hydro_motors_msg.control[_param_hy_lmotor_idx.get() - 1] = x_opt[1][1]; // 左水翼电机

		hydro_servos_msg.control[_param_hy_r_sv_idx.get() - 1] = x_opt[0][0]; // 右水翼舵机
		hydro_servos_msg.control[_param_hy_l_sv_idx.get() - 1] = x_opt[1][0];
		hydro_servos_msg.control[_param_hy_htail_sv_idx.get() - 1] = x_opt[2][0];

		_hydro_motors_pub.publish(hydro_motors_msg);
		_hydro_servos_pub.publish(hydro_servos_msg);


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

int HydroControlAllocator::print_status()
{
	PX4_INFO("Running");

	// Print current allocation method
	// switch (_allocation_method_id) {
	// case AllocationMethod::NONE:
	// 	PX4_INFO("Method: None");
	// 	break;

	// case AllocationMethod::PSEUDO_INVERSE:
	// 	PX4_INFO("Method: Pseudo-inverse");
	// 	break;

	// case AllocationMethod::SEQUENTIAL_DESATURATION:
	// 	PX4_INFO("Method: Sequential desaturation");
	// 	break;

	// case AllocationMethod::AUTO:
	// 	PX4_INFO("Method: Auto");
	// 	break;
	// }

	// Print current effectiveness matrix
	if (_hy_effectiveness.isAllFinite()) {
		PX4_INFO("_hy_effectiveness =");
		for(int i = 0; i < 5; i++){
			printf("%2u|", i); // print row numbering
			for(int j = 0; j < HY_NUM_FORCE_COMPS; j++){
				printf("% 6.5f ",(double)_hy_effectiveness(i, j));
			}
			printf("\n");
		}

	}

	// if (_handled_motor_failure_bitmask) {
	// 	PX4_INFO("Failed motors: %i (0x%x)", math::countSetBits(_handled_motor_failure_bitmask),
	// 		 _handled_motor_failure_bitmask);
	// }

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
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
