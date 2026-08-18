/****************************************************************************
 *
 *   Copyright (c) 2026 LeCAR Lab. All rights reserved.
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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "am_tilted_hex_dynamics.hpp"

namespace
{

constexpr float RotorRadius = .3683f;
constexpr float MaxThrust = 26.5127f;
constexpr float MaxTorque = 1.32564f;

void expectVectorNear(const matrix::Vector3f &actual, const matrix::Vector3f &expected, float tolerance = 1.e-5f)
{
	for (int i = 0; i < 3; ++i) {
		EXPECT_NEAR(actual(i), expected(i), tolerance);
	}
}

} // namespace

TEST(AmTiltedHexDynamics, ZeroInputProducesZeroWrench)
{
	const float inputs[am_tilted_hex::NumRotors] {};
	const am_tilted_hex::Wrench wrench =
		am_tilted_hex::computeWrench(inputs, RotorRadius, MaxThrust, MaxTorque);

	expectVectorNear(wrench.force, matrix::Vector3f{});
	expectVectorNear(wrench.moment, matrix::Vector3f{});
}

TEST(AmTiltedHexDynamics, UniformInputCancelsHorizontalWrench)
{
	const float inputs[am_tilted_hex::NumRotors] {.5f, .5f, .5f, .5f, .5f, .5f};
	const am_tilted_hex::Wrench wrench =
		am_tilted_hex::computeWrench(inputs, RotorRadius, MaxThrust, MaxTorque);

	EXPECT_NEAR(wrench.force(0), 0.f, 1.e-4f);
	EXPECT_NEAR(wrench.force(1), 0.f, 1.e-4f);
	EXPECT_NEAR(wrench.force(2), -6.f * .8660254f * MaxThrust * .25f, 1.e-3f);
	expectVectorNear(wrench.moment, matrix::Vector3f{}, 1.e-4f);
}

TEST(AmTiltedHexDynamics, MotorZeroMatchesLockedAxisSpinAndPosition)
{
	const float inputs[am_tilted_hex::NumRotors] {1.f, 0.f, 0.f, 0.f, 0.f, 0.f};
	const am_tilted_hex::Wrench wrench =
		am_tilted_hex::computeWrench(inputs, RotorRadius, MaxThrust, MaxTorque);

	const matrix::Vector3f axis{.5f, 0.f, -.866025f};
	const matrix::Vector3f normalized_axis = axis.normalized();
	const matrix::Vector3f expected_force = normalized_axis * MaxThrust;
	const matrix::Vector3f position{0.f, RotorRadius, 0.f};
	const matrix::Vector3f expected_moment = position.cross(expected_force) + normalized_axis * MaxTorque;

	expectVectorNear(wrench.force, expected_force);
	expectVectorNear(wrench.moment, expected_moment);
}
