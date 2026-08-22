#!/usr/bin/env python3
"""Audit the Aerial Manipulator FMUv4 firmware build without hardware access."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path


FLASH_BYTES = 2032 * 1024
SRAM_BYTES = 192 * 1024
MAX_FLASH_FRACTION = 0.80
MAX_STATIC_SRAM_FRACTION = 0.20
MAX_SERIAL_FRACTION = 0.80
SERIAL_BAUD = 921_600
SERIAL_BYTES_PER_SECOND = SERIAL_BAUD // 10  # 8-N-1
PROTOCOL_RESERVE = 1.25

REQUIRED_CONFIG = {
    "CONFIG_BOARD_CONSTRAINED_MEMORY": "y",
    "CONFIG_BOARD_NO_SDCARD": "n",
    "CONFIG_BOARD_SERIAL_TEL1": '"/dev/ttyS1"',
    "CONFIG_DRIVERS_ADC_BOARD_ADC": "y",
    "CONFIG_COMMON_BAROMETERS": "y",
    "CONFIG_DRIVERS_CDCACM_AUTOSTART": "y",
    "CONFIG_DRIVERS_GPS": "y",
    "CONFIG_DRIVERS_IMU_INVENSENSE_ICM20602": "y",
    "CONFIG_DRIVERS_IMU_INVENSENSE_ICM20608G": "y",
    "CONFIG_DRIVERS_IMU_INVENSENSE_MPU9250": "y",
    "CONFIG_COMMON_MAGNETOMETER": "y",
    "CONFIG_DRIVERS_PWM_OUT": "y",
    "CONFIG_DRIVERS_RC_INPUT": "y",
    "CONFIG_DRIVERS_SAFETY_BUTTON": "y",
    "CONFIG_DRIVERS_TONE_ALARM": "y",
    "CONFIG_MODULES_BATTERY_STATUS": "y",
    "CONFIG_MODULES_COMMANDER": "y",
    "CONFIG_MODULES_CONTROL_ALLOCATOR": "y",
    "CONFIG_MODULES_DATAMAN": "y",
    "CONFIG_MODULES_EKF2": "y",
    "CONFIG_MODULES_EVENTS": "y",
    "CONFIG_MODULES_FLIGHT_MODE_MANAGER": "y",
    "CONFIG_MODULES_GYRO_CALIBRATION": "y",
    "CONFIG_MODULES_LAND_DETECTOR": "y",
    "CONFIG_MODULES_LOAD_MON": "y",
    "CONFIG_MODULES_LOGGER": "y",
    "CONFIG_MODULES_MANUAL_CONTROL": "y",
    "CONFIG_MODULES_MAVLINK": "y",
    "CONFIG_MODULES_MC_ATT_CONTROL": "y",
    "CONFIG_MODULES_MC_HOVER_THRUST_ESTIMATOR": "y",
    "CONFIG_MODULES_MC_POS_CONTROL": "y",
    "CONFIG_MODULES_MC_RATE_CONTROL": "y",
    "CONFIG_MODULES_NAVIGATOR": "y",
    "CONFIG_MODULES_RC_UPDATE": "y",
    "CONFIG_MODULES_SENSORS": "y",
    "CONFIG_MODULES_UXRCE_DDS_CLIENT": "y",
    "CONFIG_SYSTEMCMDS_ACTUATOR_TEST": "y",
    "CONFIG_SYSTEMCMDS_BL_UPDATE": "y",
    "CONFIG_SYSTEMCMDS_HARDFAULT_LOG": "y",
    "CONFIG_SYSTEMCMDS_PARAM": "y",
    "CONFIG_SYSTEMCMDS_REBOOT": "y",
    "CONFIG_SYSTEMCMDS_TOP": "y",
    "CONFIG_SYSTEMCMDS_UORB": "y",
    "CONFIG_SYSTEMCMDS_USB_CONNECTED": "y",
    "CONFIG_SYSTEMCMDS_VER": "y",
    "CONFIG_SYSTEMCMDS_WORK_QUEUE": "y",
}

REMOVED_ROOT_CONFIG = {
    "CONFIG_DRIVERS_CAMERA_CAPTURE",
    "CONFIG_DRIVERS_CAMERA_TRIGGER",
    "CONFIG_COMMON_DIFFERENTIAL_PRESSURE",
    "CONFIG_COMMON_DISTANCE_SENSOR",
    "CONFIG_DRIVERS_GNSS_SEPTENTRIO",
    "CONFIG_DRIVERS_IMU_ANALOG_DEVICES_ADIS16448",
    "CONFIG_DRIVERS_IMU_INVENSENSE_ICM20948",
    "CONFIG_DRIVERS_IRLOCK",
    "CONFIG_DRIVERS_PCA9685_PWM_OUT",
    "CONFIG_DRIVERS_PWM_INPUT",
    "CONFIG_MODULES_AIRSPEED_SELECTOR",
    "CONFIG_MODULES_CAMERA_FEEDBACK",
    "CONFIG_MODULES_FW_ATT_CONTROL",
    "CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL",
    "CONFIG_MODULES_FW_MODE_MANAGER",
    "CONFIG_MODULES_FW_LATERAL_LONGITUDINAL_CONTROL",
    "CONFIG_MODULES_FW_RATE_CONTROL",
    "CONFIG_MODULES_GIMBAL",
    "CONFIG_MODULES_LANDING_TARGET_ESTIMATOR",
    "CONFIG_MODULES_LOCAL_POSITION_ESTIMATOR",
    "CONFIG_MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL",
    "CONFIG_MODULES_UUV_ATT_CONTROL",
    "CONFIG_MODULES_UUV_POS_CONTROL",
    "CONFIG_MODULES_VTOL_ATT_CONTROL",
    "CONFIG_EXAMPLES_FAKE_GPS",
}

EXPECTED_AIRFRAME_PARAMS = {
    "MAV_TYPE": "13",
    "CA_AIRFRAME": "0",
    "CA_METHOD": "0",
    "CA_ROTOR_COUNT": "6",
    "CA_ROTOR0_PX": "0",
    "CA_ROTOR0_PY": "1",
    "CA_ROTOR0_AX": "0.5",
    "CA_ROTOR0_AY": "0",
    "CA_ROTOR0_AZ": "-0.866025",
    "CA_ROTOR0_CT": "1",
    "CA_ROTOR0_KM": "-0.05",
    "CA_ROTOR1_PX": "0",
    "CA_ROTOR1_PY": "-1",
    "CA_ROTOR1_AX": "0.5",
    "CA_ROTOR1_AY": "0",
    "CA_ROTOR1_AZ": "-0.866025",
    "CA_ROTOR1_CT": "1",
    "CA_ROTOR1_KM": "0.05",
    "CA_ROTOR2_PX": "0.866025",
    "CA_ROTOR2_PY": "-0.5",
    "CA_ROTOR2_AX": "-0.25",
    "CA_ROTOR2_AY": "-0.4330125",
    "CA_ROTOR2_AZ": "-0.866025",
    "CA_ROTOR2_CT": "1",
    "CA_ROTOR2_KM": "-0.05",
    "CA_ROTOR3_PX": "-0.866025",
    "CA_ROTOR3_PY": "0.5",
    "CA_ROTOR3_AX": "-0.25",
    "CA_ROTOR3_AY": "-0.4330125",
    "CA_ROTOR3_AZ": "-0.866025",
    "CA_ROTOR3_CT": "1",
    "CA_ROTOR3_KM": "0.05",
    "CA_ROTOR4_PX": "0.866025",
    "CA_ROTOR4_PY": "0.5",
    "CA_ROTOR4_AX": "-0.25",
    "CA_ROTOR4_AY": "0.4330125",
    "CA_ROTOR4_AZ": "-0.866025",
    "CA_ROTOR4_CT": "1",
    "CA_ROTOR4_KM": "0.05",
    "CA_ROTOR5_PX": "-0.866025",
    "CA_ROTOR5_PY": "-0.5",
    "CA_ROTOR5_AX": "-0.25",
    "CA_ROTOR5_AY": "0.4330125",
    "CA_ROTOR5_AZ": "-0.866025",
    "CA_ROTOR5_CT": "1",
    "CA_ROTOR5_KM": "-0.05",
    **{f"PWM_MAIN_FUNC{i}": str(100 + i) for i in range(1, 7)},
    **{f"PWM_MAIN_DIS{i}": "900" for i in range(1, 7)},
    **{f"PWM_MAIN_MIN{i}": "1075" for i in range(1, 7)},
    **{f"PWM_MAIN_MAX{i}": "1950" for i in range(1, 7)},
    "MAV_0_CONFIG": "0",
    "SER_TEL1_BAUD": "921600",
    "UXRCE_DDS_CFG": "101",
}

# Payload-only steady-state assumptions. Event/ack topics are sporadic. The
# report applies a separate 25% protocol/framing reserve and still requires a
# hardware throughput/dropout measurement before flight.
OUTBOUND_CORE_HZ = {
    "battery_status": 1,
    "estimator_status_flags": 5,
    "failsafe_flags": 5,
    "manual_control_setpoint": 25,
    "position_setpoint_triplet": 5,
    "sensor_combined": 200,
    "timesync_status": 10,
    "vehicle_land_detected": 5,
    "vehicle_attitude": 50,
    "vehicle_control_mode": 50,
    "vehicle_local_position": 50,
    "vehicle_odometry": 100,
    "vehicle_status": 5,
    "home_position": 5,
    "input_rc": 10,
}
OUTBOUND_GPS_HZ = {"vehicle_global_position": 50, "sensor_gps": 50}
INBOUND_CONTROL_HZ = {
    "vehicle_odometry": 100,  # one mocap/visual odometry input
    "vehicle_rates_setpoint": 100,
    "offboard_control_mode": 20,
    "vehicle_command": 5,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--baseline-build-dir", type=Path)
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def read_config(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text().splitlines():
        match = re.fullmatch(r"(CONFIG_[A-Z0-9_]+)=(.*)", line)
        if match:
            values[match.group(1)] = match.group(2)
            continue
        match = re.fullmatch(r"# (CONFIG_[A-Z0-9_]+) is not set", line)
        if match:
            values[match.group(1)] = "n"
    return values


def find_one(directory: Path, pattern: str) -> Path:
    paths = sorted(directory.glob(pattern))
    if len(paths) != 1:
        raise RuntimeError(f"expected one {pattern} in {directory}, found {len(paths)}")
    return paths[0]


def elf_size(elf: Path) -> dict[str, int | float | str]:
    tool = shutil.which("arm-none-eabi-size")
    if not tool:
        raise RuntimeError("arm-none-eabi-size is not installed")
    lines = subprocess.check_output([tool, str(elf)], text=True).strip().splitlines()
    fields = lines[-1].split()
    text_bytes, data_bytes, bss_bytes = map(int, fields[:3])
    flash = text_bytes + data_bytes
    static_sram = data_bytes + bss_bytes
    return {
        "elf": str(elf),
        "text_bytes": text_bytes,
        "data_bytes": data_bytes,
        "bss_bytes": bss_bytes,
        "flash_bytes": flash,
        "flash_fraction": flash / FLASH_BYTES,
        "static_sram_bytes": static_sram,
        "static_sram_fraction": static_sram / SRAM_BYTES,
    }


def read_airframe_params(path: Path) -> dict[str, str]:
    params: dict[str, str] = {}
    for line in path.read_text().splitlines():
        match = re.match(r"^param set-default ([A-Z0-9_]+) (\S+)", line)
        if match:
            if match.group(1) in params:
                raise RuntimeError(f"duplicate airframe parameter {match.group(1)}")
            params[match.group(1)] = match.group(2)
    return params


def read_builtin_stacks(path: Path) -> dict[str, int]:
    stacks: dict[str, int] = {}
    expression = re.compile(r'^\{ "([^"]+)", .*?, (\d+) \* ')
    for line in path.read_text().splitlines():
        match = expression.match(line)
        if match:
            stacks[match.group(1)] = int(match.group(2))
    return stacks


def topic_size(build_dir: Path, topic: str) -> int:
    header = build_dir / "uORB" / "ucdr" / f"{topic}.h"
    match = re.search(
        rf"ucdr_topic_size_{re.escape(topic)}\(\).*?return (\d+);",
        header.read_text(),
        flags=re.DOTALL,
    )
    if not match:
        raise RuntimeError(f"could not read serialized size for {topic}")
    return int(match.group(1))


def bandwidth_profile(build_dir: Path, rates: dict[str, int]) -> dict[str, object]:
    topics = {
        topic: {"serialized_bytes": topic_size(build_dir, topic), "rate_hz": rate}
        for topic, rate in rates.items()
    }
    payload = sum(item["serialized_bytes"] * item["rate_hz"] for item in topics.values())
    return {"topics": topics, "payload_bytes_per_second": payload}


def main() -> int:
    args = parse_args()
    repo = Path(__file__).resolve().parents[3]
    build_dir = args.build_dir.resolve()
    checks: list[dict[str, object]] = []

    def check(name: str, passed: bool, detail: str) -> None:
        checks.append({"name": name, "passed": passed, "detail": detail})

    config = read_config(build_dir / "boardconfig")
    mismatches = {
        name: {"expected": expected, "actual": config.get(name, "missing")}
        for name, expected in REQUIRED_CONFIG.items()
        if config.get(name) != expected
    }
    check("required configuration", not mismatches, json.dumps(mismatches, sort_keys=True))

    retained = {name: config.get(name, "missing") for name in sorted(REMOVED_ROOT_CONFIG) if config.get(name) != "n"}
    check("unrelated roots removed", not retained, json.dumps(retained, sort_keys=True))

    firmware = find_one(build_dir, "*.px4")
    size = elf_size(find_one(build_dir, "*.elf"))
    check(
        "flash margin",
        size["flash_fraction"] <= MAX_FLASH_FRACTION,
        f'{size["flash_bytes"]} / {FLASH_BYTES} ({size["flash_fraction"]:.2%})',
    )
    check(
        "static SRAM margin",
        size["static_sram_fraction"] <= MAX_STATIC_SRAM_FRACTION,
        f'{size["static_sram_bytes"]} / {SRAM_BYTES} ({size["static_sram_fraction"]:.2%})',
    )
    check("firmware artifact", firmware.stat().st_size > 0, str(firmware))

    airframe = repo / "ROMFS/px4fmu_common/init.d/airframes/6100_am_tilted_hex"
    airframe_params = read_airframe_params(airframe)
    airframe_mismatches = {
        name: {"expected": expected, "actual": airframe_params.get(name, "missing")}
        for name, expected in EXPECTED_AIRFRAME_PARAMS.items()
        if airframe_params.get(name) != expected
    }
    check("airframe contract", not airframe_mismatches, json.dumps(airframe_mismatches, sort_keys=True))
    simulation_airframe = repo / "ROMFS/px4fmu_common/init.d-posix/airframes/22002_sihsim_am_tilted_hex"
    simulation_params = read_airframe_params(simulation_airframe)
    parity_names = {
        name
        for name in EXPECTED_AIRFRAME_PARAMS
        if name.startswith("CA_ROTOR") or name.startswith("PWM_MAIN_FUNC") or name == "CA_METHOD"
    }
    parity_mismatches = {
        name: {"hardware": airframe_params.get(name), "simulation": simulation_params.get(name)}
        for name in sorted(parity_names)
        if airframe_params.get(name) != simulation_params.get(name)
    }
    check("simulation geometry parity", not parity_mismatches, json.dumps(parity_mismatches, sort_keys=True))
    check(
        "airframe packaged",
        (build_dir / "etc/init.d/airframes/6100_am_tilted_hex").is_file(),
        "SYS_AUTOSTART=6100 file present in ROMFS staging",
    )
    check(
        "no force-set parameters",
        not re.search(r"^param set (?!default )", airframe.read_text(), flags=re.MULTILINE),
        "airframe changes defaults only",
    )

    stacks = read_builtin_stacks(build_dir / "NuttX/px4.bdat")
    check(
        "uXRCE-DDS stack declaration",
        stacks.get("uxrce_dds_client") == 9000,
        f'{stacks.get("uxrce_dds_client", "missing")} bytes',
    )

    if config.get("CONFIG_MODULES_UXRCE_DDS_CLIENT") == "y":
        outbound_core = bandwidth_profile(build_dir, OUTBOUND_CORE_HZ)
        outbound_gps = bandwidth_profile(build_dir, OUTBOUND_GPS_HZ)
        inbound = bandwidth_profile(build_dir, INBOUND_CONTROL_HZ)
        outbound_payload = (
            outbound_core["payload_bytes_per_second"] + outbound_gps["payload_bytes_per_second"]
        )
        outbound_reserved = round(outbound_payload * PROTOCOL_RESERVE)
        inbound_reserved = round(inbound["payload_bytes_per_second"] * PROTOCOL_RESERVE)
    else:
        outbound_core = {"error": "uXRCE-DDS client not built"}
        outbound_gps = {"error": "uXRCE-DDS client not built"}
        inbound = {"error": "uXRCE-DDS client not built"}
        outbound_reserved = SERIAL_BYTES_PER_SECOND
        inbound_reserved = SERIAL_BYTES_PER_SECOND
    check(
        "expected outbound serial budget",
        outbound_reserved <= SERIAL_BYTES_PER_SECOND * MAX_SERIAL_FRACTION,
        f"{outbound_reserved} reserved B/s / {SERIAL_BYTES_PER_SECOND} B/s",
    )
    check(
        "expected inbound serial budget",
        inbound_reserved <= SERIAL_BYTES_PER_SECOND * MAX_SERIAL_FRACTION,
        f"{inbound_reserved} reserved B/s / {SERIAL_BYTES_PER_SECOND} B/s",
    )

    baseline: dict[str, object] | None = None
    config_diff: dict[str, list[str]] | None = None
    if args.baseline_build_dir:
        baseline_dir = args.baseline_build_dir.resolve()
        baseline = elf_size(find_one(baseline_dir, "*.elf"))
        baseline_config = read_config(baseline_dir / "boardconfig")
        enabled = {name for name, value in config.items() if value == "y"}
        baseline_enabled = {name for name, value in baseline_config.items() if value == "y"}
        config_diff = {
            "enabled_only_in_am": sorted(enabled - baseline_enabled),
            "disabled_from_upstream_default": sorted(baseline_enabled - enabled),
        }

    report = {
        "status": "pass" if all(item["passed"] for item in checks) else "fail",
        "limits": {
            "flash_bytes": FLASH_BYTES,
            "max_flash_fraction": MAX_FLASH_FRACTION,
            "sram_bytes": SRAM_BYTES,
            "max_static_sram_fraction": MAX_STATIC_SRAM_FRACTION,
            "max_expected_serial_fraction": MAX_SERIAL_FRACTION,
            "serial_baud": SERIAL_BAUD,
            "serial_bytes_per_second_8n1_each_direction": SERIAL_BYTES_PER_SECOND,
            "protocol_reserve_multiplier": PROTOCOL_RESERVE,
        },
        "checks": checks,
        "firmware": str(firmware),
        "size": size,
        "baseline_size": baseline,
        "config_diff": config_diff,
        "largest_builtin_entry_stacks": sorted(stacks.items(), key=lambda item: (-item[1], item[0]))[:12],
        "bandwidth": {
            "outbound_core": outbound_core,
            "outbound_optional_gps": outbound_gps,
            "outbound_with_gps_and_reserve_bytes_per_second": outbound_reserved,
            "inbound_mocap_rate_control": inbound,
            "inbound_with_reserve_bytes_per_second": inbound_reserved,
            "scope": (
                "Expected steady-state serialized payload, not a measured wire rate. "
                "Event traffic, XRCE framing, retries, and scheduling require a bench test."
            ),
        },
    }

    output = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output)
    print(output, end="")
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
