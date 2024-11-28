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

#include "tracking_differentiator.hpp"


TrackingDifferentiator::TrackingDifferentiator(float r, float h) :
	_r0(r),
	_h0(h)
{
	_x1_pre = 0.0f;
	_x2_pre = 0.0f;
}

void TrackingDifferentiator::update(float input)
{

	float h = 0.01;

	float x1k = _x1_pre + h * _x2_pre;
	float x2k = _x2_pre + h * fhan(_x1_pre - input, _x2_pre, _r0, _h0);
	_x1_pre = x1k;
	_x2_pre = x2k;

}

float TrackingDifferentiator::fhan(float x1, float x2, float r, float h)
{
	float d = r * powf(h, 2);
	float a0 = h * x2;
	float y = x1 + a0;
	float a1 = sqrt(d * (d + 8.f * fabsf(y)));
	float a2 = a0 + matrix::sign(y) * (a1 - d) / 2.f;
	float a = ( a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
	float u = -r * (a / d) * fsg(a, d) - r * matrix::sign(a) * (1 - fsg(a, d));

	return u;
}

float TrackingDifferentiator::fsg(float x, float d)
{
	float u = (matrix::sign(x + d) - matrix::sign(x - d)) / 2.f;
	return u;
}
