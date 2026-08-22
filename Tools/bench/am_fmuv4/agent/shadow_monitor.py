#!/usr/bin/env python3
"""Subscribe-only PX4 DDS health observer for the LAB-02A bench gate."""

from __future__ import annotations

import argparse
import json
import math
import signal
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import rclpy
from px4_msgs.msg import (
    BatteryStatus,
    FailsafeFlags,
    InputRc,
    SensorCombined,
    TimesyncStatus,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


TOPICS: dict[str, type[Any]] = {
    "/fmu/out/battery_status": BatteryStatus,
    "/fmu/out/failsafe_flags": FailsafeFlags,
    "/fmu/out/input_rc": InputRc,
    "/fmu/out/sensor_combined": SensorCombined,
    "/fmu/out/timesync_status": TimesyncStatus,
    "/fmu/out/vehicle_attitude": VehicleAttitude,
    "/fmu/out/vehicle_local_position": VehicleLocalPosition,
    "/fmu/out/vehicle_odometry": VehicleOdometry,
    "/fmu/out/vehicle_status": VehicleStatus,
}

REQUIRED = {
    "/fmu/out/battery_status",
    "/fmu/out/failsafe_flags",
    "/fmu/out/input_rc",
    "/fmu/out/sensor_combined",
    "/fmu/out/timesync_status",
    "/fmu/out/vehicle_attitude",
    "/fmu/out/vehicle_status",
}

# Each threshold is deliberately half or less of the configured firmware
# publication limit. The bench is checking gross serial/DDS degradation, not
# attempting to certify real-time control quality from host arrival time.
QUALITY_LIMITS = {
    "/fmu/out/battery_status": {"min_rate_hz": 0.5, "max_gap_s": 2.5},
    "/fmu/out/failsafe_flags": {"min_rate_hz": 2.0, "max_gap_s": 0.75},
    "/fmu/out/input_rc": {"min_rate_hz": 5.0, "max_gap_s": 0.4},
    "/fmu/out/sensor_combined": {"min_rate_hz": 100.0, "max_gap_s": 0.1},
    "/fmu/out/timesync_status": {"min_rate_hz": 5.0, "max_gap_s": 0.4},
    "/fmu/out/vehicle_attitude": {"min_rate_hz": 25.0, "max_gap_s": 0.2},
    "/fmu/out/vehicle_status": {"min_rate_hz": 2.0, "max_gap_s": 0.75},
}


@dataclass
class TopicStats:
    count: int = 0
    first_monotonic: float | None = None
    last_monotonic: float | None = None
    max_gap_s: float = 0.0

    def update(self, now: float) -> None:
        if self.first_monotonic is None:
            self.first_monotonic = now
        if self.last_monotonic is not None:
            self.max_gap_s = max(self.max_gap_s, now - self.last_monotonic)
        self.last_monotonic = now
        self.count += 1

    def report(self, stop: float) -> dict[str, float | int | None]:
        result = asdict(self)
        if self.first_monotonic is None:
            result.update({"rate_hz": 0.0, "age_s": None})
            return result
        span = max((self.last_monotonic or stop) - self.first_monotonic, 1e-9)
        result.update(
            {
                "rate_hz": (self.count - 1) / span if self.count > 1 else 0.0,
                "age_s": stop - (self.last_monotonic or stop),
            }
        )
        return result


class ShadowObserver(Node):
    def __init__(self) -> None:
        super().__init__("am_fw02_shadow_observer")
        self.stats = {topic: TopicStats() for topic in TOPICS}
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._subscriptions_hold = []
        for topic, message_type in TOPICS.items():
            subscription = self.create_subscription(
                message_type,
                topic,
                lambda _message, topic=topic: self.stats[topic].update(time.monotonic()),
                qos,
            )
            self._subscriptions_hold.append(subscription)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Observe PX4 DDS output topics without creating any publisher or service client."
    )
    parser.add_argument("--duration", type=float, default=120.0, help="Observation time in seconds")
    parser.add_argument("--output", type=Path, help="Optional JSON evidence path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        raise SystemExit("--duration must be a positive finite number")

    rclpy.init()
    observer = ShadowObserver()
    interrupted = False

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    started = time.monotonic()
    try:
        while not interrupted and time.monotonic() - started < args.duration:
            rclpy.spin_once(observer, timeout_sec=0.25)
    finally:
        stopped = time.monotonic()
        topic_reports = {topic: stats.report(stopped) for topic, stats in observer.stats.items()}
        report = {
            "schema": "am-fw02-dds-shadow-v1",
            "duration_requested_s": args.duration,
            "duration_observed_s": stopped - started,
            "interrupted": interrupted,
            "subscriber_only": True,
            "topics": topic_reports,
        }
        missing = sorted(topic for topic in REQUIRED if observer.stats[topic].count == 0)
        quality_failures = []
        for topic, limits in QUALITY_LIMITS.items():
            if observer.stats[topic].count == 0:
                continue
            measured = topic_reports[topic]
            if float(measured["rate_hz"]) < limits["min_rate_hz"]:
                quality_failures.append(
                    f"{topic}: rate {measured['rate_hz']:.3f} Hz < {limits['min_rate_hz']:.3f} Hz"
                )
            if float(measured["max_gap_s"]) > limits["max_gap_s"]:
                quality_failures.append(
                    f"{topic}: max gap {measured['max_gap_s']:.3f} s > {limits['max_gap_s']:.3f} s"
                )
        report["required_topics"] = sorted(REQUIRED)
        report["quality_limits"] = QUALITY_LIMITS
        report["missing_required_topics"] = missing
        report["quality_failures"] = quality_failures
        report["pass"] = not interrupted and not missing and not quality_failures
        output = json.dumps(report, indent=2, sort_keys=True)
        print(output)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(output + "\n", encoding="utf-8")
        observer.destroy_node()
        rclpy.shutdown()
    return 0 if report["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
