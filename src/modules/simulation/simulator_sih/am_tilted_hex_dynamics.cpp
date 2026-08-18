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

#include "am_tilted_hex_dynamics.hpp"

namespace am_tilted_hex
{

struct Rotor {
	float position[3];
	float axis[3];
	float moment_ratio_sign;
};

// Compatibility-locked order: mid-right, mid-left, front-left, rear-right,
// front-right, rear-left. Position magnitudes are normalized here and scaled
// by the measured radius at runtime.
static constexpr Rotor Rotors[NumRotors] {
	{{0.f, 1.f, 0.f}, {.5f, 0.f, -.866025f}, -1.f},
	{{0.f, -1.f, 0.f}, {.5f, 0.f, -.866025f}, 1.f},
	{{.866025f, -.5f, 0.f}, {-.25f, -.4330125f, -.866025f}, -1.f},
	{{-.866025f, .5f, 0.f}, {-.25f, -.4330125f, -.866025f}, 1.f},
	{{.866025f, .5f, 0.f}, {-.25f, .4330125f, -.866025f}, 1.f},
	{{-.866025f, -.5f, 0.f}, {-.25f, .4330125f, -.866025f}, -1.f},
};

Wrench computeWrench(const float motor_inputs[NumRotors], float rotor_radius,
		     float max_thrust, float max_reaction_torque)
{
	Wrench wrench{};

	for (int i = 0; i < NumRotors; ++i) {
		matrix::Vector3f position{Rotors[i].position[0], Rotors[i].position[1], Rotors[i].position[2]};
		matrix::Vector3f axis{Rotors[i].axis[0], Rotors[i].axis[1], Rotors[i].axis[2]};
		position *= rotor_radius;
		axis.normalize();

		const float input_squared = motor_inputs[i] * motor_inputs[i];
		const matrix::Vector3f rotor_force = axis * max_thrust * input_squared;
		const matrix::Vector3f reaction_moment =
			-axis * Rotors[i].moment_ratio_sign * max_reaction_torque * input_squared;

		wrench.force += rotor_force;
		wrench.moment += position.cross(rotor_force) + reaction_moment;
	}

	return wrench;
}

} // namespace am_tilted_hex
