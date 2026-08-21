#!/usr/bin/env python3
"""Run aerial-manipulator lifecycle and fault gates in PX4 SITL.

This harness starts an isolated PX4 rootfs, drives commands over MAVLink, and
checks the externally observable lifecycle:

    ready -> Arm ACK -> armed -> Takeoff ACK -> airborne/hold
          -> Land ACK -> AUTO.LAND -> landed -> automatic disarm

The SAFE fault suite exercises only PX4-observable boundaries. ROS lease
identities, controller activation epochs, and lost transport acknowledgements
belong to the future gateway/runtime integration tests and are not simulated
here.

It supports the custom SIH and Gazebo Harmonic models. It is simulation-only;
there is deliberately no serial-device, upload, flash, or hardware path.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Iterable, Optional

try:
    from pymavlink import mavutil  # type: ignore[import-not-found]
    from pyulog import ULog  # type: ignore[import-not-found]
except ImportError as exc:
    print(
        "ERROR: pymavlink and pyulog are required. Use the PX4 virtual "
        "environment or install Tools/setup/requirements.txt.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


NAN = float("nan")
MAV_RESULT_ACCEPTED = mavutil.mavlink.MAV_RESULT_ACCEPTED
MAV_RESULT_IN_PROGRESS = mavutil.mavlink.MAV_RESULT_IN_PROGRESS
MAV_RESULT_TEMPORARILY_REJECTED = mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
MAV_LANDED_STATE_ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
MAV_LANDED_STATE_IN_AIR = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
MAV_MODE_FLAG_SAFETY_ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
ARMING_STATE_ARMED = 2
MAVLINK_PORT = 14540
GAZEBO_TRANSPORT_IP = "127.0.0.1"
OFFBOARD_SETPOINT_INTERVAL_S = 0.10
MANUAL_CONTROL_INTERVAL_S = 0.10
OFFBOARD_POSITION_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

FAULT_SCENARIOS = (
    "arm-grounded",
    "in-air-disarm-denied",
    "offboard-loss",
    "gcs-override",
    "land-from-offboard",
    "post-land-disarm",
)
SCENARIO_GROUPS = {
    "nominal": ("nominal",),
    "safe-faults": FAULT_SCENARIOS,
    "all": ("nominal", *FAULT_SCENARIOS),
}


class LifecycleError(RuntimeError):
    """A lifecycle gate failed or timed out."""


@dataclass(frozen=True)
class Simulator:
    name: str
    build_target: str
    sim_model: str
    startup_timeout_s: float


SIMULATORS = {
    "sih": Simulator(
        name="sih",
        build_target="px4_sitl_sih",
        sim_model="sihsim_am_tilted_hex",
        startup_timeout_s=45.0,
    ),
    "gazebo": Simulator(
        name="gazebo",
        build_target="px4_sitl_default",
        sim_model="gz_am_tilted_hex",
        startup_timeout_s=75.0,
    ),
}


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def result_name(result: int) -> str:
    entry = mavutil.mavlink.enums["MAV_RESULT"].get(result)
    return entry.name if entry is not None else str(result)


def command_result_satisfies(result: int, expected_results: tuple[int, ...]) -> bool:
    if result == MAV_RESULT_IN_PROGRESS:
        return False
    return result in expected_results


def parameter_name(raw: Any) -> str:
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", errors="replace")
    return str(raw).rstrip("\x00")


def selected_scenarios(selection: str) -> tuple[str, ...]:
    return SCENARIO_GROUPS.get(selection, (selection,))


def heartbeat_is_armed(heartbeat: Any) -> bool:
    return bool(heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)


def heartbeat_mode(heartbeat: Any) -> str:
    custom_main_mode = (int(heartbeat.custom_mode) >> 16) & 0xFF
    if (
        int(heartbeat.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_PX4
        and custom_main_mode == mavutil.PX4_CUSTOM_MAIN_MODE_OFFBOARD
    ):
        # Current PX4 SITL reports Offboard without every legacy
        # auto_mode_flags bit that pymavlink's generic decoder requires.
        return "OFFBOARD"
    return str(mavutil.mode_string_v10(heartbeat))


def is_native_land_mode(mode: str) -> bool:
    return mode.upper() in {"LAND", "AUTO.LAND"}


def hold_metrics(samples: Iterable[tuple[float, float, float, float]]) -> dict[str, float]:
    points = list(samples)
    if len(points) < 2:
        raise LifecycleError("hold gate has fewer than two position samples")

    x0, y0, _, _ = points[0]
    xy_radius = max(math.hypot(x - x0, y - y0) for x, y, _, _ in points)
    zs = [z for _, _, z, _ in points]
    max_abs_vz = max(abs(vz) for _, _, _, vz in points)
    return {
        "sample_count": float(len(points)),
        "xy_radius_m": xy_radius,
        "z_span_m": max(zs) - min(zs),
        "max_abs_vz_m_s": max_abs_vz,
        "mean_altitude_m": -sum(zs) / len(zs),
    }


def assert_hold_metrics(metrics: dict[str, float]) -> None:
    limits = {
        "xy_radius_m": 0.75,
        "z_span_m": 0.35,
        "max_abs_vz_m_s": 0.50,
    }
    failures = [
        f"{key}={metrics[key]:.3f} > {limit:.3f}"
        for key, limit in limits.items()
        if metrics[key] > limit
    ]
    if failures:
        raise LifecycleError("hold gate failed: " + ", ".join(failures))


def assert_containment_metrics(metrics: dict[str, float]) -> None:
    limits = {
        "xy_radius_m": 1.50,
        "z_span_m": 0.75,
        "max_abs_vz_m_s": 0.75,
    }
    failures = [
        f"{key}={metrics[key]:.3f} > {limit:.3f}"
        for key, limit in limits.items()
        if metrics[key] > limit
    ]
    if failures:
        raise LifecycleError("containment gate failed: " + ", ".join(failures))


def assert_grounded_metrics(metrics: dict[str, float]) -> None:
    limits = {
        "xy_radius_m": 0.15,
        "z_span_m": 0.10,
        "max_abs_vz_m_s": 0.20,
    }
    failures = [
        f"{key}={metrics[key]:.3f} > {limit:.3f}"
        for key, limit in limits.items()
        if metrics[key] > limit
    ]
    if failures:
        raise LifecycleError("armed-grounded gate failed: " + ", ".join(failures))


def assert_estimator_grounded_metrics(metrics: dict[str, float]) -> None:
    limits = {
        "xy_radius_m": 0.30,
        "z_span_m": 0.30,
        "max_abs_vz_m_s": 0.35,
    }
    failures = [
        f"{key}={metrics[key]:.3f} > {limit:.3f}"
        for key, limit in limits.items()
        if metrics[key] > limit
    ]
    if failures:
        raise LifecycleError("armed-grounded estimator gate failed: " + ", ".join(failures))


class RunRecord:
    def __init__(self, simulator: str, scenario: str, artifact_dir: Path) -> None:
        self.start_monotonic = time.monotonic()
        self.data: dict[str, Any] = {
            "schema": "am-tilted-hex-lifecycle-v2",
            "simulator": simulator,
            "scenario": scenario,
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "artifact_dir": str(artifact_dir),
            "events": [],
            "result": "RUNNING",
        }

    def event(self, name: str, **details: Any) -> None:
        elapsed = time.monotonic() - self.start_monotonic
        item = {"name": name, "elapsed_s": round(elapsed, 3), **details}
        self.data["events"].append(item)
        detail_text = " ".join(f"{key}={value}" for key, value in details.items())
        print(f"[{elapsed:7.2f}s] {name}" + (f"  {detail_text}" if detail_text else ""), flush=True)


class MavlinkLifecycle:
    def __init__(self, mav: Any, record: RunRecord, process: subprocess.Popen[Any]) -> None:
        self.mav = mav
        self.record = record
        self.process = process
        self.latest: dict[str, Any] = {}
        self.acks: list[Any] = []
        self.param_values: list[Any] = []
        self.status_text: list[str] = []
        self.next_heartbeat = 0.0
        self.offboard_target: Optional[tuple[float, float, float]] = None
        self.next_offboard_setpoint = 0.0
        self.neutral_manual_control_enabled = False
        self.next_manual_control = 0.0

    def send_gcs_heartbeat(self) -> None:
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        self.next_heartbeat = time.monotonic() + 0.75

    def update(self, message: Any) -> None:
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return

        if message_type == "HEARTBEAT":
            if message.get_srcSystem() != self.mav.target_system:
                return
            if message.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                return

        self.latest[message_type] = message
        if message_type == "COMMAND_ACK":
            self.acks.append(message)
        elif message_type == "PARAM_VALUE":
            self.param_values.append(message)
        elif message_type == "STATUSTEXT":
            text = str(message.text).rstrip("\x00")
            if not self.status_text or self.status_text[-1] != text:
                self.status_text.append(text)
                self.status_text = self.status_text[-20:]

    def poll(self, timeout: float = 0.20) -> Optional[Any]:
        returncode = self.process.poll()
        if returncode is not None:
            raise LifecycleError(f"PX4 exited unexpectedly with return code {returncode}")
        if time.monotonic() >= self.next_heartbeat:
            self.send_gcs_heartbeat()
        if self.offboard_target is not None and time.monotonic() >= self.next_offboard_setpoint:
            self.send_offboard_position()
        if self.neutral_manual_control_enabled and time.monotonic() >= self.next_manual_control:
            self.send_neutral_manual_control()
        message = self.mav.recv_match(blocking=True, timeout=timeout)
        if message is not None:
            self.update(message)
        return message

    def send_offboard_position(self) -> None:
        if self.offboard_target is None:
            return
        x, y, z = self.offboard_target
        self.mav.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            OFFBOARD_POSITION_TYPE_MASK,
            x,
            y,
            z,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self.next_offboard_setpoint = time.monotonic() + OFFBOARD_SETPOINT_INTERVAL_S

    def start_offboard_position(self, x: float, y: float, z: float) -> None:
        self.offboard_target = (x, y, z)
        self.next_offboard_setpoint = 0.0
        self.send_offboard_position()
        self.record.event("offboard_setpoint_stream_started", x=round(x, 4), y=round(y, 4), z=round(z, 4))

    def stop_offboard_position(self, reason: str) -> None:
        self.offboard_target = None
        self.record.event("offboard_setpoint_stream_stopped", reason=reason)

    def send_neutral_manual_control(self) -> None:
        self.mav.mav.manual_control_send(
            self.mav.target_system,
            0,
            0,
            500,
            0,
            0,
        )
        self.next_manual_control = time.monotonic() + MANUAL_CONTROL_INTERVAL_S

    def start_neutral_manual_control(self) -> None:
        self.neutral_manual_control_enabled = True
        self.next_manual_control = 0.0
        self.send_neutral_manual_control()
        self.record.event("neutral_manual_control_stream_started")

    def wait_for(
        self,
        label: str,
        predicate: Callable[[dict[str, Any]], bool],
        timeout_s: float,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate(self.latest):
                self.record.event(label)
                return
            self.poll()
        status = self.status_text[-3:]
        heartbeat = self.latest.get("HEARTBEAT")
        mode = heartbeat_mode(heartbeat) if heartbeat is not None else "unavailable"
        heartbeat_fields = (
            f", base_mode={int(heartbeat.base_mode)}, custom_mode={int(heartbeat.custom_mode)}"
            if heartbeat is not None
            else ""
        )
        raise LifecycleError(
            f"timeout waiting for {label}; latest_mode={mode}{heartbeat_fields}; "
            f"recent STATUSTEXT={status}"
        )

    def request_interval(self, message_id: int, rate_hz: float) -> None:
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id),
            1_000_000.0 / rate_hz,
            0,
            0,
            0,
            0,
            0,
        )

    def command(
        self,
        name: str,
        command: int,
        params: tuple[float, float, float, float, float, float, float],
        timeout_s: float = 8.0,
        expected_results: tuple[int, ...] = (MAV_RESULT_ACCEPTED,),
    ) -> Any:
        first_new_ack = len(self.acks)
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            command,
            0,
            *params,
        )
        self.record.event(f"{name}_sent", command=command)

        deadline = time.monotonic() + timeout_s
        inspected = first_new_ack
        while time.monotonic() < deadline:
            self.poll()
            while inspected < len(self.acks):
                ack = self.acks[inspected]
                inspected += 1
                if int(ack.command) != command:
                    continue
                result = int(ack.result)
                self.record.event(
                    f"{name}_ack",
                    command=command,
                    result=result_name(result),
                )
                if result == MAV_RESULT_IN_PROGRESS:
                    continue
                if not command_result_satisfies(result, expected_results):
                    expectation = ", ".join(result_name(item) for item in expected_results)
                    raise LifecycleError(
                        f"{name} returned {result_name(result)}, expected one of {expectation}; "
                        f"recent STATUSTEXT={self.status_text[-3:]}"
                    )
                return ack

        raise LifecycleError(
            f"timeout waiting for {name} COMMAND_ACK; "
            f"recent STATUSTEXT={self.status_text[-3:]}"
        )

    def read_param_float(self, name: str, timeout_s: float = 5.0) -> float:
        first_new_value = len(self.param_values)
        self.mav.mav.param_request_read_send(
            self.mav.target_system,
            self.mav.target_component,
            name.encode("ascii"),
            -1,
        )
        deadline = time.monotonic() + timeout_s
        inspected = first_new_value
        while time.monotonic() < deadline:
            self.poll()
            while inspected < len(self.param_values):
                value = self.param_values[inspected]
                inspected += 1
                if parameter_name(value.param_id) == name:
                    return float(value.param_value)
        raise LifecycleError(f"timeout reading parameter {name}")

    def set_param_float(self, name: str, value: float, timeout_s: float = 5.0) -> None:
        first_new_value = len(self.param_values)
        self.mav.mav.param_set_send(
            self.mav.target_system,
            self.mav.target_component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.monotonic() + timeout_s
        inspected = first_new_value
        seen: list[float] = []
        while time.monotonic() < deadline:
            self.poll()
            while inspected < len(self.param_values):
                message = self.param_values[inspected]
                inspected += 1
                if parameter_name(message.param_id) != name:
                    continue
                observed = float(message.param_value)
                seen.append(observed)
                if math.isclose(observed, value, rel_tol=0.0, abs_tol=1e-5):
                    self.record.event("parameter_set", parameter=name, value=value)
                    return
        raise LifecycleError(f"parameter {name} did not become {value}; observed={seen}")

    def pump_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.poll(timeout=min(0.20, deadline - time.monotonic()))


def repository_root() -> Path:
    return Path(__file__).resolve().parents[3]


def running_px4_pids() -> list[int]:
    pids: list[int] = []
    for proc_dir in Path("/proc").glob("[0-9]*"):
        try:
            if (proc_dir / "comm").read_text().strip() == "px4":
                pids.append(int(proc_dir.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return pids


def gazebo_environment(base: Optional[dict[str, str]] = None) -> dict[str, str]:
    """Keep the isolated Gazebo server and clients on the local host.

    Gazebo Transport otherwise selects a non-loopback interface. That can
    make local service calls unreachable on networks that use CGNAT address
    space or block traffic back to the host's own interface address.
    """
    env = dict(os.environ if base is None else base)
    env["GZ_IP"] = GAZEBO_TRANSPORT_IP
    return env


def assert_udp_port_available(port: int) -> None:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", port))
    except OSError as exc:
        raise LifecycleError(f"UDP port {port} is unavailable: {exc}") from exc
    finally:
        probe.close()


def assert_clean_start(simulator: Simulator, port: int) -> None:
    pids = running_px4_pids()
    if pids:
        raise LifecycleError(f"PX4 is already running (pids {pids}); stop it before the test")
    assert_udp_port_available(port)

    if simulator.name == "gazebo" and shutil.which("gz"):
        topics = subprocess.run(
            ["gz", "topic", "-l"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
            env=gazebo_environment(),
        ).stdout.splitlines()
        worlds = [topic for topic in topics if topic.startswith("/world/") and topic.endswith("/clock")]
        if worlds:
            raise LifecycleError(
                "Gazebo already has an active world; stop the interactive simulation "
                f"before this isolated test ({worlds})"
            )


def make_environment(repo: Path) -> dict[str, str]:
    env = dict(os.environ)
    venv_bin = repo / ".venv" / "bin"
    if venv_bin.is_dir():
        env["PATH"] = f"{venv_bin}:{env.get('PATH', '')}"
    return env


def build_simulator(repo: Path, simulator: Simulator, artifact_dir: Path) -> None:
    env = make_environment(repo)
    command = ["make", simulator.build_target]
    log_path = artifact_dir / "build.log"
    print("Building: " + " ".join(command), flush=True)
    with log_path.open("w") as log:
        result = subprocess.run(
            command,
            cwd=repo,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    if result.returncode != 0:
        raise LifecycleError(f"build failed; see {log_path}")


def prepare_rootfs(repo: Path, simulator: Simulator, artifact_dir: Path) -> tuple[Path, Path]:
    build_dir = repo / "build" / simulator.build_target
    px4_binary = build_dir / "bin" / "px4"
    etc_dir = build_dir / "etc"
    if not px4_binary.is_file() or not etc_dir.is_dir():
        raise LifecycleError(f"missing {simulator.build_target} build products")

    rootfs = artifact_dir / "rootfs"
    rootfs.mkdir(parents=True)
    (rootfs / "etc").symlink_to(etc_dir)
    (rootfs / "test_data").symlink_to(repo / "test_data")

    if simulator.name == "gazebo":
        gz_env = build_dir / "rootfs" / "gz_env.sh"
        if not gz_env.is_file():
            raise LifecycleError(f"missing generated Gazebo environment: {gz_env}")
        (rootfs / "gz_env.sh").symlink_to(gz_env)

    return rootfs, px4_binary


def stop_process_group(process: subprocess.Popen[Any], record: RunRecord) -> None:
    if process.poll() is not None:
        record.event("px4_exited", returncode=process.returncode)
        return

    for sig, wait_s in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=wait_s)
            break
        except subprocess.TimeoutExpired:
            continue
    record.event("px4_stopped", returncode=process.poll())


def newest_ulog(rootfs: Path) -> Optional[Path]:
    logs = list((rootfs / "log").glob("**/*.ulg"))
    return max(logs, key=lambda path: path.stat().st_mtime) if logs else None


def inspect_arm_grounded_ulog(ulog: ULog) -> dict[str, float]:
    datasets = {dataset.name: dataset.data for dataset in ulog.data_list}
    status = datasets.get("vehicle_status")
    groundtruth = datasets.get("vehicle_local_position_groundtruth")
    if status is None or groundtruth is None:
        raise LifecycleError("armed-grounded ULog is missing status or groundtruth")

    armed_start: Optional[int] = None
    armed_end: Optional[int] = None
    for timestamp, arming_state in zip(status["timestamp"], status["arming_state"]):
        if int(arming_state) == ARMING_STATE_ARMED and armed_start is None:
            armed_start = int(timestamp)
        elif armed_start is not None and int(arming_state) != ARMING_STATE_ARMED:
            armed_end = int(timestamp)
            break
    if armed_start is None:
        raise LifecycleError("armed-grounded ULog has no observed armed interval")
    if armed_end is None:
        armed_end = int(groundtruth["timestamp"][-1])

    samples = [
        (float(x), float(y), float(z), float(vz))
        for timestamp, x, y, z, vz in zip(
            groundtruth["timestamp"],
            groundtruth["x"],
            groundtruth["y"],
            groundtruth["z"],
            groundtruth["vz"],
        )
        if armed_start <= int(timestamp) <= armed_end
    ]
    metrics = hold_metrics(samples)
    assert_grounded_metrics(metrics)
    return {key: round(value, 4) for key, value in metrics.items()}


def inspect_ulog(path: Path, scenario: str) -> dict[str, Any]:
    ulog = ULog(str(path))
    dropouts = list(ulog.dropouts)
    topics = {dataset.name for dataset in ulog.data_list}
    required_topics = {
        "vehicle_command",
        "vehicle_command_ack",
        "vehicle_land_detected",
        "vehicle_status",
    }
    missing = sorted(required_topics - topics)
    if missing:
        raise LifecycleError(f"ULog is missing lifecycle topics: {missing}")
    if dropouts:
        total_ms = sum(int(dropout.duration) for dropout in dropouts)
        raise LifecycleError(f"ULog has {len(dropouts)} dropouts totaling {total_ms} ms")
    result: dict[str, Any] = {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "dropout_count": 0,
        "required_topics": sorted(required_topics),
    }
    if scenario == "arm-grounded":
        result["armed_grounded_physical_metrics"] = inspect_arm_grounded_ulog(ulog)
    return result


def collect_position_metrics(lifecycle: MavlinkLifecycle, duration_s: float) -> dict[str, float]:
    samples: list[tuple[float, float, float, float]] = []
    seen_time_boot_ms: Optional[int] = None
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        lifecycle.poll()
        position = lifecycle.latest.get("LOCAL_POSITION_NED")
        if position is None or int(position.time_boot_ms) == seen_time_boot_ms:
            continue
        seen_time_boot_ms = int(position.time_boot_ms)
        samples.append((float(position.x), float(position.y), float(position.z), float(position.vz)))

    return hold_metrics(samples)


def collect_hold(lifecycle: MavlinkLifecycle, duration_s: float) -> dict[str, float]:
    metrics = collect_position_metrics(lifecycle, duration_s)
    assert_hold_metrics(metrics)
    lifecycle.record.event("hold_gate_passed", **{key: round(value, 4) for key, value in metrics.items()})
    return metrics


def collect_containment(lifecycle: MavlinkLifecycle, duration_s: float) -> dict[str, float]:
    metrics = collect_position_metrics(lifecycle, duration_s)
    assert_containment_metrics(metrics)
    lifecycle.record.event(
        "containment_gate_passed",
        **{key: round(value, 4) for key, value in metrics.items()},
    )
    return metrics


def collect_armed_grounded(lifecycle: MavlinkLifecycle, duration_s: float) -> dict[str, float]:
    deadline = time.monotonic() + duration_s
    samples: list[tuple[float, float, float, float]] = []
    seen_time_boot_ms: Optional[int] = None
    while time.monotonic() < deadline:
        lifecycle.poll()
        extended = lifecycle.latest.get("EXTENDED_SYS_STATE")
        if extended is not None and extended.landed_state == MAV_LANDED_STATE_IN_AIR:
            raise LifecycleError("vehicle became airborne before an explicit Takeoff")
        position = lifecycle.latest.get("LOCAL_POSITION_NED")
        if position is None or int(position.time_boot_ms) == seen_time_boot_ms:
            continue
        seen_time_boot_ms = int(position.time_boot_ms)
        samples.append((float(position.x), float(position.y), float(position.z), float(position.vz)))

    metrics = hold_metrics(samples)
    # SIH's estimator can drift on the ground even when simulator groundtruth
    # and PX4's land detector remain fixed. Keep a bounded online sanity check;
    # inspect_ulog applies the tighter physical-motion gate to groundtruth.
    assert_estimator_grounded_metrics(metrics)
    heartbeat = lifecycle.latest.get("HEARTBEAT")
    extended = lifecycle.latest.get("EXTENDED_SYS_STATE")
    if heartbeat is None or not heartbeat_is_armed(heartbeat):
        raise LifecycleError("vehicle did not remain armed during grounded gate")
    if extended is None or extended.landed_state != MAV_LANDED_STATE_ON_GROUND:
        raise LifecycleError("vehicle did not remain landed during grounded gate")
    lifecycle.record.event(
        "armed_grounded_gate_passed",
        **{key: round(value, 4) for key, value in metrics.items()},
    )
    return metrics


def prepare_vehicle(lifecycle: MavlinkLifecycle) -> None:
    lifecycle.request_interval(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5.0)
    lifecycle.request_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20.0)
    lifecycle.request_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5.0)
    lifecycle.request_interval(mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 5.0)

    lifecycle.wait_for(
        "initial_on_ground",
        lambda latest: (
            latest.get("EXTENDED_SYS_STATE") is not None
            and latest["EXTENDED_SYS_STATE"].landed_state == MAV_LANDED_STATE_ON_GROUND
        ),
        20.0,
    )
    lifecycle.wait_for(
        "local_position_ready",
        lambda latest: latest.get("LOCAL_POSITION_NED") is not None,
        20.0,
    )
    lifecycle.wait_for(
        "global_position_ready",
        lambda latest: latest.get("GLOBAL_POSITION_INT") is not None,
        30.0,
    )
    lifecycle.pump_for(2.0)


def arm_vehicle(lifecycle: MavlinkLifecycle) -> None:
    lifecycle.command(
        "arm",
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    lifecycle.wait_for(
        "armed_observed",
        lambda latest: latest.get("HEARTBEAT") is not None and heartbeat_is_armed(latest["HEARTBEAT"]),
        10.0,
    )


def disarm_vehicle(lifecycle: MavlinkLifecycle, name: str = "disarm") -> None:
    lifecycle.command(
        name,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    lifecycle.wait_for(
        "disarmed_observed",
        lambda latest: (
            latest.get("HEARTBEAT") is not None
            and not heartbeat_is_armed(latest["HEARTBEAT"])
        ),
        10.0,
    )


def takeoff_vehicle(
    lifecycle: MavlinkLifecycle,
    takeoff_altitude_m: float,
    hold_s: float,
) -> None:
    global_position = lifecycle.latest["GLOBAL_POSITION_INT"]
    target_alt_msl_m = float(global_position.alt) / 1000.0 + takeoff_altitude_m
    lifecycle.command(
        "takeoff",
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        (NAN, 0.0, 0.0, NAN, NAN, NAN, target_alt_msl_m),
    )
    lifecycle.wait_for(
        "in_air_observed",
        lambda latest: (
            latest.get("EXTENDED_SYS_STATE") is not None
            and latest["EXTENDED_SYS_STATE"].landed_state == MAV_LANDED_STATE_IN_AIR
        ),
        20.0,
    )
    lifecycle.wait_for(
        "takeoff_altitude_reached",
        lambda latest: (
            latest.get("LOCAL_POSITION_NED") is not None
            and abs(-float(latest["LOCAL_POSITION_NED"].z) - takeoff_altitude_m) <= 0.20
            and abs(float(latest["LOCAL_POSITION_NED"].vz)) <= 0.15
        ),
        45.0,
    )
    # Do not score the tail end of the takeoff ramp as hold performance. The
    # position/velocity predicate above establishes proximity; this short
    # dwell establishes that it persists before the fixed observation window.
    lifecycle.pump_for(1.0)
    collect_hold(lifecycle, hold_s)


def request_mode(lifecycle: MavlinkLifecycle, name: str, mode: str) -> None:
    mode_values = mavutil.px4_map[mode]
    lifecycle.command(
        name,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        (
            float(mode_values[0]),
            float(mode_values[1]),
            float(mode_values[2]),
            0.0,
            0.0,
            0.0,
            0.0,
        ),
    )


def enter_offboard_hold(lifecycle: MavlinkLifecycle, hold_s: float) -> None:
    position = lifecycle.latest.get("LOCAL_POSITION_NED")
    if position is None:
        raise LifecycleError("local position unavailable before Offboard entry")
    lifecycle.start_offboard_position(float(position.x), float(position.y), float(position.z))
    lifecycle.pump_for(1.25)
    request_mode(lifecycle, "offboard", "OFFBOARD")
    lifecycle.wait_for(
        "offboard_mode_observed",
        lambda latest: (
            latest.get("HEARTBEAT") is not None
            and heartbeat_mode(latest["HEARTBEAT"]).upper() == "OFFBOARD"
        ),
        10.0,
    )
    collect_hold(lifecycle, hold_s)


def land_vehicle(
    lifecycle: MavlinkLifecycle,
    wait_for_auto_disarm: bool = True,
    stop_offboard_at_handoff: bool = False,
) -> None:
    lifecycle.command(
        "land",
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        (0.0, 0.0, 0.0, NAN, NAN, NAN, NAN),
    )
    lifecycle.wait_for(
        "native_land_mode_observed",
        lambda latest: (
            latest.get("HEARTBEAT") is not None
            and is_native_land_mode(heartbeat_mode(latest["HEARTBEAT"]))
        ),
        10.0,
    )
    if stop_offboard_at_handoff:
        lifecycle.stop_offboard_position("px4_native_land_observed")
    lifecycle.wait_for(
        "landed_observed",
        lambda latest: (
            latest.get("EXTENDED_SYS_STATE") is not None
            and latest["EXTENDED_SYS_STATE"].landed_state == MAV_LANDED_STATE_ON_GROUND
        ),
        75.0,
    )
    if wait_for_auto_disarm:
        lifecycle.wait_for(
            "automatic_disarm_observed",
            lambda latest: (
                latest.get("HEARTBEAT") is not None
                and not heartbeat_is_armed(latest["HEARTBEAT"])
            ),
            20.0,
        )
    lifecycle.pump_for(1.0)


def run_nominal(lifecycle: MavlinkLifecycle, takeoff_altitude_m: float, hold_s: float) -> None:
    prepare_vehicle(lifecycle)
    arm_vehicle(lifecycle)
    takeoff_vehicle(lifecycle, takeoff_altitude_m, hold_s)
    land_vehicle(lifecycle)


def run_arm_grounded(lifecycle: MavlinkLifecycle, takeoff_altitude_m: float, hold_s: float) -> None:
    del takeoff_altitude_m, hold_s
    prepare_vehicle(lifecycle)
    arm_vehicle(lifecycle)
    collect_armed_grounded(lifecycle, 3.0)
    disarm_vehicle(lifecycle, "grounded_disarm")
    lifecycle.pump_for(1.0)


def run_in_air_disarm_denied(
    lifecycle: MavlinkLifecycle,
    takeoff_altitude_m: float,
    hold_s: float,
) -> None:
    prepare_vehicle(lifecycle)
    arm_vehicle(lifecycle)
    takeoff_vehicle(lifecycle, takeoff_altitude_m, hold_s)
    ack = lifecycle.command(
        "in_air_disarm",
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        expected_results=(MAV_RESULT_TEMPORARILY_REJECTED,),
    )
    heartbeat = lifecycle.latest.get("HEARTBEAT")
    extended = lifecycle.latest.get("EXTENDED_SYS_STATE")
    if heartbeat is None or not heartbeat_is_armed(heartbeat):
        raise LifecycleError("in-air Disarm rejection did not preserve armed truth")
    if extended is None or extended.landed_state != MAV_LANDED_STATE_IN_AIR:
        raise LifecycleError("in-air Disarm rejection did not preserve airborne truth")
    lifecycle.record.event(
        "in_air_disarm_rejection_preserved_truth",
        result=result_name(int(ack.result)),
    )
    lifecycle.pump_for(1.0)
    land_vehicle(lifecycle)


def run_offboard_loss(lifecycle: MavlinkLifecycle, takeoff_altitude_m: float, hold_s: float) -> None:
    lifecycle.start_neutral_manual_control()
    prepare_vehicle(lifecycle)
    arm_vehicle(lifecycle)
    takeoff_vehicle(lifecycle, takeoff_altitude_m, hold_s)
    enter_offboard_hold(lifecycle, hold_s)
    lifecycle.stop_offboard_position("injected_source_loss")
    lifecycle.wait_for(
        "offboard_loss_position_mode_observed",
        lambda latest: (
            latest.get("HEARTBEAT") is not None
            and heartbeat_mode(latest["HEARTBEAT"]).upper() == "POSCTL"
        ),
        15.0,
    )
    collect_containment(lifecycle, max(2.0, hold_s))
    heartbeat = lifecycle.latest.get("HEARTBEAT")
    extended = lifecycle.latest.get("EXTENDED_SYS_STATE")
    if heartbeat is None or not heartbeat_is_armed(heartbeat):
        raise LifecycleError("Offboard-loss fallback did not preserve armed truth")
    if heartbeat_mode(heartbeat).upper() != "POSCTL":
        raise LifecycleError("Offboard was reacquired without a new mode request")
    if extended is None or extended.landed_state != MAV_LANDED_STATE_IN_AIR:
        raise LifecycleError("Offboard-loss Position fallback did not remain airborne")
    lifecycle.record.event("offboard_loss_no_reacquisition_gate_passed", mode="POSCTL")
    land_vehicle(lifecycle)


def run_gcs_override(lifecycle: MavlinkLifecycle, takeoff_altitude_m: float, hold_s: float) -> None:
    lifecycle.start_neutral_manual_control()
    prepare_vehicle(lifecycle)
    arm_vehicle(lifecycle)
    takeoff_vehicle(lifecycle, takeoff_altitude_m, hold_s)
    enter_offboard_hold(lifecycle, hold_s)
    request_mode(lifecycle, "gcs_position_override", "POSCTL")
    lifecycle.wait_for(
        "gcs_position_override_observed",
        lambda latest: (
            latest.get("HEARTBEAT") is not None
            and heartbeat_mode(latest["HEARTBEAT"]).upper() == "POSCTL"
        ),
        10.0,
    )
    lifecycle.pump_for(2.0)
    heartbeat = lifecycle.latest.get("HEARTBEAT")
    if heartbeat is None or heartbeat_mode(heartbeat).upper() != "POSCTL":
        raise LifecycleError("active Offboard setpoints silently reacquired external control")
    lifecycle.record.event("gcs_override_no_reacquisition_gate_passed", mode="POSCTL")
    lifecycle.stop_offboard_position("gcs_override_complete")
    collect_containment(lifecycle, max(2.0, hold_s))
    land_vehicle(lifecycle)


def run_land_from_offboard(
    lifecycle: MavlinkLifecycle,
    takeoff_altitude_m: float,
    hold_s: float,
) -> None:
    prepare_vehicle(lifecycle)
    arm_vehicle(lifecycle)
    takeoff_vehicle(lifecycle, takeoff_altitude_m, hold_s)
    enter_offboard_hold(lifecycle, hold_s)
    land_vehicle(lifecycle, stop_offboard_at_handoff=True)


def run_post_land_disarm(
    lifecycle: MavlinkLifecycle,
    takeoff_altitude_m: float,
    hold_s: float,
) -> None:
    prepare_vehicle(lifecycle)
    original = lifecycle.read_param_float("COM_DISARM_LAND")
    lifecycle.record.event("parameter_observed", parameter="COM_DISARM_LAND", value=original)
    lifecycle.set_param_float("COM_DISARM_LAND", -1.0)
    arm_vehicle(lifecycle)
    takeoff_vehicle(lifecycle, takeoff_altitude_m, hold_s)
    land_vehicle(lifecycle, wait_for_auto_disarm=False)
    lifecycle.pump_for(3.0)
    heartbeat = lifecycle.latest.get("HEARTBEAT")
    extended = lifecycle.latest.get("EXTENDED_SYS_STATE")
    if heartbeat is None or not heartbeat_is_armed(heartbeat):
        raise LifecycleError("auto-disarm-disabled vehicle did not remain armed after landing")
    if extended is None or extended.landed_state != MAV_LANDED_STATE_ON_GROUND:
        raise LifecycleError("post-Land Disarm scenario lost landed truth")
    lifecycle.record.event("auto_disarm_disabled_gate_passed")
    disarm_vehicle(lifecycle, "post_land_disarm")
    lifecycle.record.event("one_shot_post_land_disarm_gate_passed")
    lifecycle.pump_for(1.0)


SCENARIO_RUNNERS: dict[str, Callable[[MavlinkLifecycle, float, float], None]] = {
    "nominal": run_nominal,
    "arm-grounded": run_arm_grounded,
    "in-air-disarm-denied": run_in_air_disarm_denied,
    "offboard-loss": run_offboard_loss,
    "gcs-override": run_gcs_override,
    "land-from-offboard": run_land_from_offboard,
    "post-land-disarm": run_post_land_disarm,
}


def git_revision(repo: Path, path: Optional[str] = None) -> str:
    command = ["git", "rev-parse", "HEAD"] if path is None else ["git", "rev-parse", f"HEAD:{path}"]
    return subprocess.check_output(command, cwd=repo, text=True).strip()


def run_one(
    repo: Path,
    simulator: Simulator,
    scenario: str,
    args: argparse.Namespace,
    base_artifact_dir: Path,
    build: bool,
) -> dict[str, Any]:
    artifact_dir = base_artifact_dir / f"{utc_stamp()}_{simulator.name}_{scenario}"
    artifact_dir.mkdir(parents=True)
    record = RunRecord(simulator.name, scenario, artifact_dir)
    record.data["px4_revision"] = git_revision(repo)
    record.data["gazebo_model_revision"] = git_revision(repo, "Tools/simulation/gz")
    record.data["build_performed"] = build
    record.data["build_passed"] = not build
    process: Optional[subprocess.Popen[Any]] = None
    mav: Optional[Any] = None
    px4_log: Optional[Any] = None

    try:
        assert_clean_start(simulator, MAVLINK_PORT)
        record.event("clean_start_gate_passed", mavlink_port=MAVLINK_PORT)

        if build:
            build_simulator(repo, simulator, artifact_dir)
            record.data["build_passed"] = True
            record.event("build_passed", target=simulator.build_target)
        else:
            record.event("build_reused", target=simulator.build_target)

        rootfs, px4_binary = prepare_rootfs(repo, simulator, artifact_dir)
        env = make_environment(repo)
        env["PX4_SIM_MODEL"] = simulator.sim_model
        if simulator.name == "gazebo":
            env = gazebo_environment(env)
            record.event("gazebo_transport_isolated", ip=GAZEBO_TRANSPORT_IP)
            if not args.gui:
                env["HEADLESS"] = "1"
            else:
                env.pop("HEADLESS", None)

        px4_log_path = artifact_dir / "px4.log"
        px4_log = px4_log_path.open("w")
        process = subprocess.Popen(
            [str(px4_binary), "-d", str(repo / "build" / simulator.build_target / "etc")],
            cwd=rootfs,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=px4_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        record.event("px4_started", pid=process.pid, log=str(px4_log_path))

        connection = f"udpin:127.0.0.1:{MAVLINK_PORT}"
        mav = mavutil.mavlink_connection(
            connection,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            autoreconnect=False,
        )
        heartbeat = mav.wait_heartbeat(timeout=int(simulator.startup_timeout_s))
        if heartbeat is None:
            raise LifecycleError(f"no PX4 heartbeat on {connection}")
        if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            raise LifecycleError(f"heartbeat on {connection} is not from PX4")
        record.event(
            "px4_heartbeat",
            system=mav.target_system,
            component=heartbeat.get_srcComponent(),
            mode=heartbeat_mode(heartbeat),
        )

        lifecycle = MavlinkLifecycle(mav, record, process)
        lifecycle.update(heartbeat)
        lifecycle.send_gcs_heartbeat()
        SCENARIO_RUNNERS[scenario](lifecycle, args.takeoff_altitude, args.hold_seconds)
        record.data["result"] = "PASS"
    except Exception as exc:  # capture complete artifacts for every gate failure
        record.data["result"] = "FAIL"
        record.data["error"] = str(exc)
        record.event("run_failed", error=str(exc))
    finally:
        if record.data["result"] == "RUNNING":
            record.data["result"] = "FAIL"
            record.data["error"] = "run interrupted"
        if mav is not None:
            mav.close()
        if process is not None:
            stop_process_group(process, record)
        if px4_log is not None:
            px4_log.close()

        rootfs = artifact_dir / "rootfs"
        ulog = newest_ulog(rootfs)
        if ulog is None:
            if record.data["result"] == "PASS":
                record.data["result"] = "FAIL"
                record.data["error"] = "no ULog produced"
        else:
            try:
                record.data["ulog"] = inspect_ulog(ulog, scenario)
                record.event("ulog_gate_passed", path=str(ulog))
            except Exception as exc:
                record.data["result"] = "FAIL"
                record.data["error"] = str(exc)
                record.event("ulog_gate_failed", error=str(exc))

        record.data["finished_utc"] = datetime.now(timezone.utc).isoformat()
        record.data["duration_s"] = round(time.monotonic() - record.start_monotonic, 3)
        summary_path = artifact_dir / "summary.json"
        summary_path.write_text(json.dumps(record.data, indent=2, sort_keys=True) + "\n")
        print(
            f"{simulator.name}/{scenario}: {record.data['result']} ({summary_path})",
            flush=True,
        )

    return record.data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--simulator",
        choices=("sih", "gazebo", "all"),
        default="all",
        help="simulator to exercise (default: all, sequentially)",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="show the Gazebo GUI; ignored for SIH",
    )
    parser.add_argument(
        "--scenario",
        choices=(*SCENARIO_RUNNERS, *SCENARIO_GROUPS),
        default="nominal",
        help="scenario or scenario group to run (default: nominal)",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="reuse existing build products",
    )
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=Path("build/am_tilted_hex_lifecycle"),
        help="artifact root relative to the PX4 checkout",
    )
    parser.add_argument(
        "--takeoff-altitude",
        type=float,
        default=2.5,
        help="takeoff height above the initial position in metres",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=4.0,
        help="wall-clock hold-observation duration",
    )
    args = parser.parse_args()
    if not 1.0 <= args.takeoff_altitude <= 10.0:
        parser.error("--takeoff-altitude must be between 1 and 10 metres")
    if not 2.0 <= args.hold_seconds <= 30.0:
        parser.error("--hold-seconds must be between 2 and 30 seconds")
    return args


def main() -> int:
    args = parse_args()
    repo = repository_root()
    base_artifact_dir = args.artifact_dir
    if not base_artifact_dir.is_absolute():
        base_artifact_dir = repo / base_artifact_dir
    base_artifact_dir.mkdir(parents=True, exist_ok=True)

    names = ("sih", "gazebo") if args.simulator == "all" else (args.simulator,)
    scenarios = selected_scenarios(args.scenario)
    results: list[dict[str, Any]] = []
    for name in names:
        simulator = SIMULATORS[name]
        for index, scenario in enumerate(scenarios):
            build = not args.skip_build and index == 0
            result = run_one(repo, simulator, scenario, args, base_artifact_dir, build)
            results.append(result)
            if build and not result["build_passed"]:
                break
    passed = all(result["result"] == "PASS" for result in results)
    print("PASS: all requested lifecycle gates" if passed else "FAIL: lifecycle gate failure")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
