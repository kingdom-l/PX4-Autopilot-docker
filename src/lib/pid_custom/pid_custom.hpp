/****************************************************************************
 *
 *   Copyright (c) 2015 Roman Bapst. All rights reserved.
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
 * @file pid_custom.hpp
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <float.h>
#include <cmath>
#include "pid_type.h"

// 预期通过其他模块调用该对象，并通过其他模块来获取px4::params，赋值给td参数
class PIDCustom
{
public:
	// PIDCustom(int pid);

	PIDCustom(PID_Improvement_e Improve) {
		memset(&_pid, 0, sizeof(_pid));
		_pid.Improve = Improve;
	}

	~PIDCustom() = default;

	void abs_limit(float *a, float ABS_MAX);

	float get_deltaT(hrt_abstime *then);

	void f_trapezoid_intergral();

	void f_changing_integration_rate();

	void f_derivative_on_measurement();

	void f_derivative_filter();

	void f_output_filter();

	float forward_feed(Forward_Feed_s *instance, float in);

	void update_parameter(float kp, float ki, float maxout, float ilimit, float ea, float eb, float fk);

	float pid_calculate(float get, float set);

	float pid_get_iout(){return _pid.Iout;}

private:

	PIDInstance _pid;
};
