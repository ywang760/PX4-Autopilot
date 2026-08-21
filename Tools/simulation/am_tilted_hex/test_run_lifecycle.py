#!/usr/bin/env python3

import unittest
from types import SimpleNamespace

from pymavlink import mavutil

from run_lifecycle import (
    FAULT_SCENARIOS,
    LifecycleError,
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_IN_PROGRESS,
    MAV_RESULT_TEMPORARILY_REJECTED,
    assert_containment_metrics,
    assert_grounded_metrics,
    assert_hold_metrics,
    command_result_satisfies,
    gazebo_environment,
    heartbeat_mode,
    hold_metrics,
    inspect_arm_grounded_ulog,
    is_native_land_mode,
    parameter_name,
    selected_scenarios,
)


class LifecycleHelpersTest(unittest.TestCase):
    def test_native_land_mode_requires_land_without_takeoff(self) -> None:
        self.assertTrue(is_native_land_mode("AUTO.LAND"))
        self.assertTrue(is_native_land_mode("LAND"))
        self.assertFalse(is_native_land_mode("AUTO.TAKEOFF"))
        self.assertFalse(is_native_land_mode("AUTO.PRECLAND"))
        self.assertFalse(is_native_land_mode("POSCTL"))

    def test_current_px4_offboard_heartbeat_is_decoded_explicitly(self) -> None:
        heartbeat = SimpleNamespace(
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_PX4,
            base_mode=145,
            custom_mode=mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16,
        )
        self.assertEqual(heartbeat_mode(heartbeat), "OFFBOARD")

    def test_hold_metrics_for_stationary_samples(self) -> None:
        metrics = hold_metrics(
            [
                (0.0, 0.0, -2.50, 0.02),
                (0.02, -0.01, -2.52, -0.01),
                (-0.01, 0.02, -2.49, 0.01),
            ]
        )
        self.assertEqual(metrics["sample_count"], 3)
        self.assertAlmostEqual(metrics["mean_altitude_m"], 2.503333, places=5)
        assert_hold_metrics(metrics)

    def test_hold_gate_rejects_each_limit(self) -> None:
        failures = [
            {"xy_radius_m": 0.76, "z_span_m": 0.1, "max_abs_vz_m_s": 0.1},
            {"xy_radius_m": 0.1, "z_span_m": 0.36, "max_abs_vz_m_s": 0.1},
            {"xy_radius_m": 0.1, "z_span_m": 0.1, "max_abs_vz_m_s": 0.51},
        ]
        for metrics in failures:
            with self.subTest(metrics=metrics):
                with self.assertRaisesRegex(LifecycleError, "hold gate failed"):
                    assert_hold_metrics(metrics)

    def test_grounded_gate_is_stricter_than_hold_gate(self) -> None:
        safe = {"xy_radius_m": 0.02, "z_span_m": 0.01, "max_abs_vz_m_s": 0.02}
        assert_grounded_metrics(safe)
        for key, value in (
            ("xy_radius_m", 0.16),
            ("z_span_m", 0.11),
            ("max_abs_vz_m_s", 0.21),
        ):
            metrics = dict(safe)
            metrics[key] = value
            with self.subTest(key=key):
                with self.assertRaisesRegex(LifecycleError, "armed-grounded gate failed"):
                    assert_grounded_metrics(metrics)

    def test_containment_gate_is_bounded_but_not_a_tuning_claim(self) -> None:
        assert_containment_metrics(
            {"xy_radius_m": 1.0, "z_span_m": 0.5, "max_abs_vz_m_s": 0.5}
        )
        with self.assertRaisesRegex(LifecycleError, "containment gate failed"):
            assert_containment_metrics(
                {"xy_radius_m": 1.51, "z_span_m": 0.5, "max_abs_vz_m_s": 0.5}
            )

    def test_command_result_expectations_separate_acceptance_and_rejection(self) -> None:
        accepted = (MAV_RESULT_ACCEPTED,)
        rejected = (MAV_RESULT_TEMPORARILY_REJECTED,)
        self.assertTrue(command_result_satisfies(MAV_RESULT_ACCEPTED, accepted))
        self.assertFalse(command_result_satisfies(MAV_RESULT_ACCEPTED, rejected))
        self.assertFalse(command_result_satisfies(MAV_RESULT_IN_PROGRESS, accepted))
        self.assertFalse(command_result_satisfies(MAV_RESULT_IN_PROGRESS, rejected))
        self.assertTrue(command_result_satisfies(MAV_RESULT_TEMPORARILY_REJECTED, rejected))

    def test_scenario_groups_are_explicit_and_stable(self) -> None:
        self.assertEqual(selected_scenarios("nominal"), ("nominal",))
        self.assertEqual(selected_scenarios("safe-faults"), FAULT_SCENARIOS)
        self.assertEqual(selected_scenarios("all"), ("nominal", *FAULT_SCENARIOS))
        self.assertEqual(selected_scenarios("offboard-loss"), ("offboard-loss",))

    def test_parameter_names_accept_mavlink_bytes_and_strings(self) -> None:
        self.assertEqual(parameter_name(b"COM_DISARM_LAND\x00"), "COM_DISARM_LAND")
        self.assertEqual(parameter_name("COM_OF_LOSS_T\x00"), "COM_OF_LOSS_T")

    def test_gazebo_transport_is_pinned_to_loopback(self) -> None:
        env = gazebo_environment({"GZ_IP": "100.110.190.102", "PATH": "/bin"})
        self.assertEqual(env["GZ_IP"], "127.0.0.1")
        self.assertEqual(env["PATH"], "/bin")

    def test_armed_grounded_ulog_gate_uses_only_the_armed_window(self) -> None:
        status = SimpleNamespace(
            name="vehicle_status",
            data={"timestamp": [0, 10, 30], "arming_state": [1, 2, 1]},
        )
        groundtruth = SimpleNamespace(
            name="vehicle_local_position_groundtruth",
            data={
                "timestamp": [0, 10, 20, 30, 40],
                "x": [9.0, 0.0, 0.01, 0.0, 9.0],
                "y": [9.0, 0.0, 0.01, 0.0, 9.0],
                "z": [9.0, 0.0, 0.01, 0.0, 9.0],
                "vz": [9.0, 0.0, 0.01, 0.0, 9.0],
            },
        )
        metrics = inspect_arm_grounded_ulog(SimpleNamespace(data_list=[status, groundtruth]))
        self.assertEqual(metrics["sample_count"], 3.0)
        self.assertLess(metrics["z_span_m"], 0.02)


if __name__ == "__main__":
    unittest.main()
