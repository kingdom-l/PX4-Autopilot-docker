/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * @file two_order_eso.cpp
 */

#include "two_order_eso.hpp"


TwoOrderEso::TwoOrderEso(float beta1, float beta2) :
	_beta1(beta1),
	_beta2(beta2)
{
	// _beta1 = 100.f;
	// _beta2 = 300.f;
	_z10 = 0.0f;
	_z20 = 0.0f;
}

void TwoOrderEso::update(float u, float y, float b)
{

	float h = 0.01;
	float e = _z10 - y;

	float z1k = _z10 + h * (_z20 + b * u - _beta1 * e);
	float z2k = _z20 - h * _beta2 * fal(e, 0.5, 0.05);
	_z10 = z1k;
	_z20 = z2k;

}

float TwoOrderEso::fal(float e, float alp, float h)
{
	float fe;
	if(fabsf(e) > h){
		fe = powf(fabsf(e), alp) * matrix::sign(e);
	}
	else{
		fe = e / powf(h, alp);
	}

	return fe;
}
