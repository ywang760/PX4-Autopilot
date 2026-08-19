#!/usr/bin/env python3

import unittest

from run_lifecycle import (
    LifecycleError,
    assert_hold_metrics,
    hold_metrics,
    is_native_land_mode,
)


class LifecycleHelpersTest(unittest.TestCase):
    def test_native_land_mode_requires_land_without_takeoff(self) -> None:
        self.assertTrue(is_native_land_mode("AUTO.LAND"))
        self.assertTrue(is_native_land_mode("LAND"))
        self.assertFalse(is_native_land_mode("AUTO.TAKEOFF"))
        self.assertFalse(is_native_land_mode("AUTO.PRECLAND"))
        self.assertFalse(is_native_land_mode("POSCTL"))

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


if __name__ == "__main__":
    unittest.main()
