/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include <math.h>
#include <zephyr/kernel.h>

#include "util.h"

float q_mag(const float *q)
{
	return sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

void q_normalize(const float *q, float *out)
{
	float mag = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (mag == 0)
		return;
	out[0] = q[0]/mag;
	out[1] = q[1]/mag;
	out[2] = q[2]/mag;
	out[3] = q[3]/mag;
}

void q_multiply(const float *x, const float *y, float *out)
{
	out[0] = x[0]*y[0] - x[1]*y[1] - x[2]*y[2] - x[3]*y[3];
	out[1] = x[1]*y[0] + x[0]*y[1] - x[3]*y[2] + x[2]*y[3];
	out[2] = x[2]*y[0] + x[3]*y[1] + x[0]*y[2] - x[1]*y[3];
	out[3] = x[3]*y[0] - x[2]*y[1] + x[1]*y[2] + x[0]*y[3];
}

void q_conj(const float *q, float *out)
{
	out[0] = q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = -q[3];
}

float q_diff_mag(const float *x, const float *y)
{
	float z[4];
	float q[4];
	q_conj(x, z);
	q_multiply(z, y, q);
	if (q[0] > 1)
		return 0;
	return fabsf(2 * acosf(q[0]));
}

bool q_epsilon(const float *x, const float *y, float eps)
{
	float z[4];
	float q[4];
	q_conj(x, z);
	q_multiply(z, y, q);
	if (q[0] > 1)
		return true;
	return fabsf(2 * acosf(q[0])) < eps;
}

// http://marc-b-reynolds.github.io/quaternions/2017/05/02/QuatQuantPart1.html#fnref:pos:3
// https://github.com/Marc-B-Reynolds/Stand-alone-junk/blob/559bd78893a3a95cdee1845834c632141b945a45/src/Posts/quatquant0.c#L898
void q_fem(const float *q, float *out)
{
	float w = fabsf(q[0]);
	float a = 1 - w * w;
	float inv_sqrt_a = 1/sqrtf(a + (float)EPS); // inversesqrt
	float k = a * inv_sqrt_a;
	float atan_term = (2 / M_PI) * atanf(k / w);
	float sign_w = (q[0] == 0) ? 1 : copysignf(1, q[0]);
	float s = atan_term * inv_sqrt_a * sign_w;
	out[0] = s * q[1];
	out[1] = s * q[2];
	out[2] = s * q[3];
}

void q_iem(const float *v, float *out)
{
	float d = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
	float inv_sqrt_d = 1/sqrtf(d + (float)EPS); // inversesqrt
	float a = (M_PI / 2) * d * inv_sqrt_d;
	float s = sinf(a);
	float k = s * inv_sqrt_d;
	out[0] = cosf(a);
	out[1] = k * v[0];
	out[2] = k * v[1];
	out[3] = k * v[2];
}
