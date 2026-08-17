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
 * @file three_order_eso.hpp
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <float.h>
// 预期通过其他模块调用该对象，并通过其他模块来获取px4::params，赋值给td参数
class ThreeOrderEso
{
public:
	ThreeOrderEso(float b0, float beta1, float beta2, float beta3, float h);
	~ThreeOrderEso() = default;

	void update(float u, float y);

	float getStateEst() const{
		return _z10;
	}

	float getStateDotEst() const{
		return _z20;
	}


	float getTotalDisturbance() const{
		return _z30;
	}

	void set_params(float b0, float beta1, float beta2, float beta3, float h){
		_b0 = b0;
		_beta1 = beta1;
		_beta2 = beta2;
		_beta3 = beta3;
		_h = h;
	}




private:

	float fal(float e, float alp, float delta);

	float _z10;
	float _z20;
	float _z30;

	float _b0;
	float _beta1;
	float _beta2;
	float _beta3;
	float _h;

};
