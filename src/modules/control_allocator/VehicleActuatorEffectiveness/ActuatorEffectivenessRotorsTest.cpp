/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessRotorsTest.cpp
 *
 * Tests for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include <gtest/gtest.h>
#include "ActuatorEffectivenessRotors.hpp"

using namespace matrix;

TEST(ActuatorEffectivenessRotors, QuadrotorX)
{
	ActuatorEffectivenessRotors::Geometry geometry = {};
	geometry.rotors[0].position = {1.f, 1.f, 0.f};
	geometry.rotors[0].axis = {0.f, 0.f, -1.f};
	geometry.rotors[0].thrust_coef = 1.0f;
	geometry.rotors[0].moment_ratio = 0.05f;

	geometry.rotors[1].position = {-1.f, -1.f, 0.f};
	geometry.rotors[1].axis = {0.f, 0.f, -1.f};
	geometry.rotors[1].thrust_coef = 1.0f;
	geometry.rotors[1].moment_ratio = 0.05f;

	geometry.rotors[2].position = {1.f, -1.f, 0.f};
	geometry.rotors[2].axis = {0.f, 0.f, -1.f};
	geometry.rotors[2].thrust_coef = 1.0f;
	geometry.rotors[2].moment_ratio = -0.05f;

	geometry.rotors[3].position = {-1.f, 1.f, 0.f};
	geometry.rotors[3].axis = {0.f, 0.f, -1.f};
	geometry.rotors[3].thrust_coef = 1.0f;
	geometry.rotors[3].moment_ratio = -0.05f;

	geometry.num_rotors = 4;

	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry, effectiveness);

	const float expected[ActuatorEffectiveness::NUM_AXES][ActuatorEffectiveness::NUM_ACTUATORS] = {
		{-1.0f,   1.0f,   1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 1.0f,  -1.0f,   1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.05f,  0.05f, -0.05f, -0.05f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-1.0f,  -1.0f,  -1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	ActuatorEffectiveness::EffectivenessMatrix effectiveness_expected(expected);

	EXPECT_EQ(effectiveness, effectiveness_expected);
}

TEST(ActuatorEffectivenessRotors, HexarotorX)
{
	ActuatorEffectivenessRotors::Geometry geometry = {};
	geometry.rotors[0].position = {0.f, .5f, 0.f};
	geometry.rotors[0].axis = {0.f, 0.f, -1.f};
	geometry.rotors[0].thrust_coef = 1.0f;
	geometry.rotors[0].moment_ratio = -0.05f;

	geometry.rotors[1].position = {0.f, -.5f, 0.f};
	geometry.rotors[1].axis = {0.f, 0.f, -1.f};
	geometry.rotors[1].thrust_coef = 1.0f;
	geometry.rotors[1].moment_ratio = 0.05f;

	geometry.rotors[2].position = {.43f, -.25f, 0.f};
	geometry.rotors[2].axis = {0.f, 0.f, -1.f};
	geometry.rotors[2].thrust_coef = 1.0f;
	geometry.rotors[2].moment_ratio = -0.05f;

	geometry.rotors[3].position = {-.43f, .25f, 0.f};
	geometry.rotors[3].axis = {0.f, 0.f, -1.f};
	geometry.rotors[3].thrust_coef = 1.0f;
	geometry.rotors[3].moment_ratio = 0.05f;

	geometry.rotors[4].position = {.43f, .25f, 0.f};
	geometry.rotors[4].axis = {0.f, 0.f, -1.f};
	geometry.rotors[4].thrust_coef = 1.0f;
	geometry.rotors[4].moment_ratio = 0.05f;

	geometry.rotors[5].position = {-.43f, -.25f, 0.f};
	geometry.rotors[5].axis = {0.f, 0.f, -1.f};
	geometry.rotors[5].thrust_coef = 1.0f;
	geometry.rotors[5].moment_ratio = -0.05f;

	geometry.num_rotors = 6;

	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry, effectiveness);

	const float expected[ActuatorEffectiveness::NUM_AXES][ActuatorEffectiveness::NUM_ACTUATORS] = {
		{-0.5f,  0.5f,   0.25f, -0.25f, -0.25f,  0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,   0.f,    0.43f, -0.43f,  0.43f, -0.43f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-0.05f, 0.05f, -0.05f,  0.05f,  0.05f, -0.05f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,   0.f,    0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,   0.f,    0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-1.f,  -1.f,   -1.f,   -1.f,   -1.f,   -1.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	ActuatorEffectiveness::EffectivenessMatrix effectiveness_expected(expected);

	EXPECT_EQ(effectiveness, effectiveness_expected);
}

TEST(ActuatorEffectivenessRotors, AerialManipulatorTiltedHex)
{
	// Flight-proven order: mid-right, mid-left, front-left, rear-right,
	// front-right, rear-left. These are normalized geometry coefficients.
	ActuatorEffectivenessRotors::Geometry geometry{};
	geometry.num_rotors = 6;

	geometry.rotors[0] = {{0.f, 1.f, 0.f}, {.5f, 0.f, -.866025f}, 1.f, -.05f, -1};
	geometry.rotors[1] = {{0.f, -1.f, 0.f}, {.5f, 0.f, -.866025f}, 1.f, .05f, -1};
	geometry.rotors[2] = {{.866025f, -.5f, 0.f}, {-.25f, -.4330125f, -.866025f}, 1.f, -.05f, -1};
	geometry.rotors[3] = {{-.866025f, .5f, 0.f}, {-.25f, -.4330125f, -.866025f}, 1.f, .05f, -1};
	geometry.rotors[4] = {{.866025f, .5f, 0.f}, {-.25f, .4330125f, -.866025f}, 1.f, .05f, -1};
	geometry.rotors[5] = {{-.866025f, -.5f, 0.f}, {-.25f, .4330125f, -.866025f}, 1.f, -.05f, -1};

	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	EXPECT_EQ(ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry, effectiveness), 6);

	const float expected[6][6] = {
		{-.841025294f, .841025294f, .420512684f, -.420512684f, -.420512684f, .420512684f},
		{0.f, 0.f, .728348994f, -.728348994f, .728348994f, -.728348994f},
		{-.543301440f, .543301440f, -.543301138f, .543301138f, .543301138f, -.543301138f},
		{.500000175f, .500000175f, -.250000109f, -.250000109f, -.250000109f, -.250000109f},
		{0.f, 0.f, -.433012689f, -.433012689f, .433012689f, .433012689f},
		{-.866025303f, -.866025303f, -.866025379f, -.866025379f, -.866025379f, -.866025379f}
	};

	for (int row = 0; row < 6; ++row) {
		for (int column = 0; column < 6; ++column) {
			EXPECT_NEAR(effectiveness(row, column), expected[row][column], 1.e-6f);
		}
	}

	// Equal commands must cancel every moment and horizontal force while
	// retaining vertical thrust. This also catches ordering/sign regressions.
	for (int row = 0; row < 5; ++row) {
		float sum = 0.f;

		for (int column = 0; column < 6; ++column) {
			sum += effectiveness(row, column);
		}

		EXPECT_NEAR(sum, 0.f, 1.e-6f);
	}

	float vertical_sum = 0.f;

	for (int column = 0; column < 6; ++column) {
		vertical_sum += effectiveness(5, column);
	}

	EXPECT_NEAR(vertical_sum, -6.f * .8660254f, 1.e-5f);

	// A successful inverse and identity product prove the locked 6x6 wrench
	// matrix is full rank in the actual PX4 matrix implementation.
	matrix::SquareMatrix<float, 6> wrench_matrix;

	for (int row = 0; row < 6; ++row) {
		for (int column = 0; column < 6; ++column) {
			wrench_matrix(row, column) = effectiveness(row, column);
		}
	}

	matrix::SquareMatrix<float, 6> inverse;
	ASSERT_TRUE(matrix::inv(wrench_matrix, inverse));
	const matrix::SquareMatrix<float, 6> identity = wrench_matrix * inverse;

	for (int row = 0; row < 6; ++row) {
		for (int column = 0; column < 6; ++column) {
			EXPECT_NEAR(identity(row, column), row == column ? 1.f : 0.f, 1.e-4f);
		}
	}
}

TEST(ActuatorEffectivenessRotors, Tilt)
{
	Vector3f axis_expected{0.f, 0.f, -1.f};
	Vector3f axis = ActuatorEffectivenessRotors::tiltedAxis(0.f, 0.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{1.f, 0.f, 0.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(M_PI_F / 2.f, 0.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{1.f / sqrtf(2.f), 0.f, -1.f / sqrtf(2.f)};
	axis = ActuatorEffectivenessRotors::tiltedAxis(M_PI_F / 2.f / 2.f, 0.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{-1.f, 0.f, 0.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(-M_PI_F / 2.f, 0.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{0.f, 0.f, -1.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(0.f, M_PI_F / 2.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{0.f, 1.f, 0.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(M_PI_F / 2.f, M_PI_F / 2.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{0.f, -1.f, 0.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(-M_PI_F / 2.f, M_PI_F / 2.f);
	EXPECT_EQ(axis, axis_expected);
}


TEST(ActuatorEffectivenessRotors, isAlignedWithCoordinateAxis)
{
	// Exactly along z axis
	Vector3f vec = {0.f, 0.f, 1.f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_TRUE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Approximately along x axis (atan(0.1) = 5.7 deg, below 10 deg tolerance)
	vec = Vector3f{1.f, 0.f, -0.1f};
	EXPECT_TRUE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Approximately along x axis (atan(0.2) = 11.3 deg, above 10 deg tolerance)
	vec = Vector3f{1.f, 0.f, -0.2f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Approximately along y axis, with both other axes slightly off
	// (atan(0.1 sqrt(2)) = 8.05 deg, below 10 deg tolerance)
	vec = Vector3f{0.1f, 1.0f, -0.1f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_TRUE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Same but with larger offset: atan(0.2 sqrt(2)) = 15.79 deg
	vec = Vector3f{0.2f, 1.0f, -0.2f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Random vector far from any axes
	vec = Vector3f{-100.f, -100.f, 100.f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

}
