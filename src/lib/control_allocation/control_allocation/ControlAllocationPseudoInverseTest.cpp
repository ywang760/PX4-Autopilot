/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationPseudoInverseTest.cpp
 *
 * Tests for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include <gtest/gtest.h>
#include <ControlAllocationPseudoInverse.hpp>

using namespace matrix;

TEST(ControlAllocationTest, AllZeroCase)
{
	ControlAllocationPseudoInverse method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	method.clipActuatorSetpoint();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
}

TEST(ControlAllocationMetricTest, AllZeroCase)
{
	ControlAllocationPseudoInverse method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	method.setMetricAllocation(true);
	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
}

class ControlAllocationPseudoInverseTestAmTiltedHex : public ::testing::Test
{
public:
	static constexpr uint8_t NUM_ACTUATORS = 6;
	using ControlVector = Vector<float, ControlAllocation::NUM_AXES>;
	using HexActuatorVector = Vector<float, NUM_ACTUATORS>;

	void SetUp() override
	{
		// Exact matrix is independently locked against ActuatorEffectivenessRotors
		// in its functional test. Keeping this unit test parameter-free isolates
		// the allocator's normalization and final clipping behavior.
		const float expected[ControlAllocation::NUM_AXES][NUM_ACTUATORS] = {
			{-.841025294f, .841025294f, .420512684f, -.420512684f, -.420512684f, .420512684f},
			{0.f, 0.f, .728348994f, -.728348994f, .728348994f, -.728348994f},
			{-.543301440f, .543301440f, -.543301138f, .543301138f, .543301138f, -.543301138f},
			{.500000175f, .500000175f, -.250000109f, -.250000109f, -.250000109f, -.250000109f},
			{0.f, 0.f, -.433012689f, -.433012689f, .433012689f, .433012689f},
			{-.866025303f, -.866025303f, -.866025379f, -.866025379f, -.866025379f, -.866025379f}
		};
		Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> effectiveness{};

		for (int row = 0; row < ControlAllocation::NUM_AXES; ++row) {
			for (int column = 0; column < NUM_ACTUATORS; ++column) {
				effectiveness(row, column) = expected[row][column];
			}
		}

		ControlAllocation::ActuatorVector trim{};
		ControlAllocation::ActuatorVector linearization_point{};

		_control_allocation.setNormalizeRPY(true);
		_control_allocation.setEffectivenessMatrix(effectiveness, trim, linearization_point, NUM_ACTUATORS,
				true /* update native v1.18 normalization */);
	}

	HexActuatorVector allocateAndClip(const ControlVector &control)
	{
		_control_allocation.setControlSetpoint(control);
		_control_allocation.allocate();
		_control_allocation.clipActuatorSetpoint();
		const ControlAllocation::ActuatorVector &outputs = _control_allocation.getActuatorSetpoint();

		for (int i = NUM_ACTUATORS; i < ControlAllocation::NUM_ACTUATORS; ++i) {
			EXPECT_FLOAT_EQ(outputs(i), 0.f);
		}

		return HexActuatorVector(outputs.slice<NUM_ACTUATORS, 1>(0, 0));
	}

	ControlAllocationPseudoInverse _control_allocation;
};

constexpr uint8_t ControlAllocationPseudoInverseTestAmTiltedHex::NUM_ACTUATORS;

TEST_F(ControlAllocationPseudoInverseTestAmTiltedHex, NativeNormalizationCollective)
{
	ControlVector control{};
	control(ControlAllocation::THRUST_Z) = -.5f;
	const HexActuatorVector outputs = allocateAndClip(control);

	for (int i = 0; i < NUM_ACTUATORS; ++i) {
		EXPECT_NEAR(outputs(i), .5f, 1.e-5f);
	}

	const ControlVector allocated = _control_allocation.getAllocatedControl();

	for (int axis = 0; axis < ControlAllocation::NUM_AXES; ++axis) {
		EXPECT_NEAR(allocated(axis), control(axis), 1.e-5f);
	}
}

TEST_F(ControlAllocationPseudoInverseTestAmTiltedHex, SixAxisInterior)
{
	ControlVector control{};
	control(ControlAllocation::ROLL) = .1f;
	control(ControlAllocation::PITCH) = -.08f;
	control(ControlAllocation::YAW) = .03f;
	control(ControlAllocation::THRUST_X) = .1f;
	control(ControlAllocation::THRUST_Y) = -.05f;
	control(ControlAllocation::THRUST_Z) = -.45f;
	const HexActuatorVector outputs = allocateAndClip(control);

	for (int i = 0; i < NUM_ACTUATORS; ++i) {
		EXPECT_GE(outputs(i), 0.f);
		EXPECT_LE(outputs(i), 1.f);
	}

	const ControlVector allocated = _control_allocation.getAllocatedControl();

	for (int axis = 0; axis < ControlAllocation::NUM_AXES; ++axis) {
		EXPECT_NEAR(allocated(axis), control(axis), 1.e-4f);
	}
}

TEST_F(ControlAllocationPseudoInverseTestAmTiltedHex, SixAxisSaturationClipsAndReportsResidual)
{
	ControlVector control{};
	control(ControlAllocation::ROLL) = .5f;
	control(ControlAllocation::PITCH) = .3f;
	control(ControlAllocation::YAW) = .2f;
	control(ControlAllocation::THRUST_X) = .5f;
	control(ControlAllocation::THRUST_Y) = .4f;
	control(ControlAllocation::THRUST_Z) = -1.f;
	const HexActuatorVector outputs = allocateAndClip(control);

	bool touches_boundary = false;

	for (int i = 0; i < NUM_ACTUATORS; ++i) {
		EXPECT_GE(outputs(i), 0.f);
		EXPECT_LE(outputs(i), 1.f);
		touches_boundary |= outputs(i) < 1.e-5f || outputs(i) > 1.f - 1.e-5f;
	}

	EXPECT_TRUE(touches_boundary);
	const ControlVector residual = control - _control_allocation.getAllocatedControl();
	float residual_squared = 0.f;

	for (int axis = 0; axis < ControlAllocation::NUM_AXES; ++axis) {
		residual_squared += residual(axis) * residual(axis);
	}

	EXPECT_GT(residual_squared, 1.e-6f);
}
