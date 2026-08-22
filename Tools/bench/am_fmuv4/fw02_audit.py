#!/usr/bin/env python3
"""Audit the FW-02 bench package and generate its complete v1.10 inventory."""

from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import io
import json
import lzma
import math
import sys
from collections import Counter
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
WORKSPACE = REPO.parent
ROOT = Path(__file__).resolve().parent
PACKAGE = ROOT / "params/am_fmuv4_lab02a.params"
INVENTORY = ROOT / "params/migration-inventory.tsv"
METADATA = REPO / "build/px4_fmu-v4_am_tilted_hex/etc/extras/parameters.json.xz"
LEGACY_DEFAULT = (
    WORKSPACE
    / "refactor_campaign/rollback/legacy-v110-1326-source-equivalent/aug11-final-711-params.params"
)
HASHES = ROOT / "artifacts/SHA256SUMS"
MANIFEST = ROOT / "artifacts/build-manifest.json"
PINS = ROOT / "agent/pins.env"

LEGACY_SHA256 = "6dcb7dc8359a7c3fafbc2f5f7b6fca1956f3c419d2ab80a5d162bfe8dacd7302"
EXPECTED_PINS = {
    "ROS_IMAGE": "docker.io/library/ros:jazzy-ros-base-noble@sha256:bab7e640bf79cd84957e4e18fcba7d87efc3385b4e3f36a32eeca01638e43206",
    "XRCE_AGENT_REF": "v2.4.3",
    "XRCE_AGENT_COMMIT": "73622810d984349b80bbac0ef55fc0b694d62222",
    "FASTCDR_COMMIT": "4611cea8cf805befc6cac8fadf0e8f75fc92405a",
    "FASTDDS_COMMIT": "f052f239538ebc0df56835fdb57f7628628010bc",
    "PX4_MSGS_REF": "release/1.18",
    "PX4_MSGS_COMMIT": "598c7aad7b2386f9406ebd2a2f841619fddc3c78",
    "ROS_DOMAIN_ID": "0",
}

# Values that define the first bench gate. Checking them here prevents an innocent
# edit to a .params file from changing the reviewed safety/transport contract.
EXPECTED_VALUES = {
    "SYS_AUTOSTART": 6100,
    "SYS_HAS_BARO": 0,
    "SYS_HAS_MAG": 0,
    "SYS_HAS_GPS": 0,
    "RC_MAP_FLTMODE": 5,
    "RC_MAP_ARM_SW": 9,
    "RC_MAP_KILL_SW": 10,
    "COM_RC_IN_MODE": 0,
    "NAV_RCL_ACT": 3,
    "COM_OBL_RC_ACT": 0,
    "COM_OF_LOSS_T": 1.0,
    "COM_DISARM_PRFLT": 10.0,
    "MAN_OVERRIDE_SPD": 1.0,
    "EKF2_EV_CTRL": 11,
    "EKF2_HGT_REF": 3,
    "EKF2_GPS_CTRL": 0,
    "EKF2_BARO_CTRL": 0,
    "EKF2_OF_CTRL": 0,
    "EKF2_RNG_CTRL": 0,
    "EKF2_MAG_TYPE": 5,
    "MAV_0_CONFIG": 0,
    "SER_TEL1_BAUD": 921600,
    "UXRCE_DDS_CFG": 101,
    "UXRCE_DDS_FLCTRL": 0,
    "UXRCE_DDS_SYNCT": 1,
    "UXRCE_DDS_SYNCC": 0,
    "SDLOG_BACKEND": 1,
    "SDLOG_MODE": 2,
    "SDLOG_PROFILE": 19,
}

FORBIDDEN_NAMES = {
    "COM_KILL_DISARM",
    "COM_RC_OVERRIDE",
    "COM_RC_STICK_OV",
    "EKF2_AID_MASK",
    "EKF2_HGT_MODE",
}
FORBIDDEN_PREFIXES = ("CAL_", "CA_", "MC_", "MPC_", "PWM_", "PWM_MAIN_", "PWM_AUX_")

SUCCESSORS = {
    "EKF2_AID_MASK": "EKF2_EV_CTRL",
    "EKF2_HGT_MODE": "EKF2_HGT_REF",
    "COM_RC_OVERRIDE": "MAN_OVERRIDE_SPD",
    "COM_RC_STICK_OV": "MAN_OVERRIDE_SPD",
}

OVERRIDE_RATIONALE = {
    "COM_DISARM_PRFLT": "Use the v1.18 10 s automatic preflight disarm instead of legacy disabled behavior.",
    "COM_OBL_RC_ACT": "Use Position as the approved default after offboard loss; physical RC remains authoritative.",
    "COM_OF_LOSS_T": "Use the v1.18 1 s timeout instead of the legacy zero-second transition.",
    "MAN_DEADZONE": "Use the v1.18 normalized deadzone and verify neutral sticks on the bench.",
    "MAN_OVERRIDE_SPD": "Use v1.18 speed-based manual override; the legacy 12 percent threshold has no numeric equivalent.",
    "MAV_0_CONFIG": "Disable TELEM1 MAVLink because that byte stream is exclusively assigned to DDS.",
    "SDLOG_BACKEND": "Use SD logging only so MAVLink logging cannot consume USB or serial bandwidth.",
    "SDLOG_MODE": "Log from boot through shutdown for the disarmed bench evidence window.",
    "SER_TEL1_BAUD": "Move TELEM1 from legacy MAVLink 57600 to dedicated DDS 921600.",
}


class Audit:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.notes: list[str] = []

    def require(self, condition: bool, message: str) -> None:
        if not condition:
            self.errors.append(message)

    def note(self, message: str) -> None:
        self.notes.append(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_params(path: Path) -> dict[str, tuple[str, int]]:
    result: dict[str, tuple[str, int]] = {}
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_number}: expected five fields")
        name, value, parameter_type = fields[2], fields[3], int(fields[4])
        if name in result:
            raise ValueError(f"{path}:{line_number}: duplicate parameter {name}")
        result[name] = (value, parameter_type)
    return result


def numeric(value: str, parameter_type: int) -> int | float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"non-finite parameter value {value}")
    if parameter_type == 6:
        if not parsed.is_integer():
            raise ValueError(f"integer parameter has non-integer value {value}")
        return int(parsed)
    if parameter_type == 9:
        return parsed
    raise ValueError(f"unsupported MAVLink parameter type {parameter_type}")


def close(left: int | float, right: int | float) -> bool:
    return math.isclose(float(left), float(right), rel_tol=1e-7, abs_tol=1e-7)


def load_metadata() -> dict[str, dict[str, object]]:
    if not METADATA.exists():
        raise FileNotFoundError(
            f"missing {METADATA}; run `make px4_fmu-v4_am_tilted_hex` before this audit"
        )
    with lzma.open(METADATA, "rt", encoding="utf-8") as stream:
        document = json.load(stream)
    return {parameter["name"]: parameter for parameter in document["parameters"]}


def validate_package(audit: Audit, package: dict[str, tuple[str, int]], metadata: dict[str, dict[str, object]]) -> None:
    audit.require(bool(package), "curated parameter package is empty")
    for name, (raw_value, parameter_type) in package.items():
        audit.require(name in metadata, f"package parameter {name} is absent from v1.18 metadata")
        if name not in metadata:
            continue
        description = metadata[name]
        expected_type = {"Int32": 6, "Float": 9}.get(str(description.get("type")))
        audit.require(parameter_type == expected_type, f"{name}: type {parameter_type}, expected {expected_type}")
        try:
            value = numeric(raw_value, parameter_type)
        except ValueError as error:
            audit.errors.append(f"{name}: {error}")
            continue
        if "min" in description:
            audit.require(float(value) >= float(description["min"]), f"{name}: {value} below minimum")
        if "max" in description:
            audit.require(float(value) <= float(description["max"]), f"{name}: {value} above maximum")
        values = description.get("values")
        if values:
            allowed = {float(item["value"]) for item in values}
            audit.require(float(value) in allowed, f"{name}: {value} is not an enumerated value")

    for name, expected in EXPECTED_VALUES.items():
        audit.require(name in package, f"required bench parameter {name} is missing")
        if name in package:
            actual = numeric(*package[name])
            audit.require(close(actual, expected), f"{name}: expected {expected}, found {actual}")

    for name in package:
        audit.require(name not in FORBIDDEN_NAMES, f"legacy-only parameter {name} must not be imported")
        audit.require(
            not name.startswith(FORBIDDEN_PREFIXES),
            f"{name} is owned by fresh calibration, v1.18 control defaults, or airframe 6100",
        )

    audit.note(f"validated {len(package)} current package values; the count is not a selection quota")


def airframe_values() -> dict[str, float]:
    airframe = REPO / "ROMFS/px4fmu_common/init.d/airframes/6100_am_tilted_hex"
    values: dict[str, float] = {}
    for line in airframe.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) != 4 or fields[:2] != ["param", "set-default"]:
            continue
        name, raw_value = fields[2], fields[3]
        if (
            name in {"SYS_AUTOSTART", "MAV_TYPE", "CA_METHOD", "CA_ROTOR_COUNT"}
            or name.startswith("CA_ROTOR")
            or name.startswith("PWM_MAIN_FUNC")
            or name.startswith("PWM_MAIN_DIS")
            or name.startswith("PWM_MAIN_MIN")
            or name.startswith("PWM_MAIN_MAX")
        ):
            values[name] = float(raw_value)
    return values


def validate_effective_export(
    audit: Audit,
    path: Path,
    package: dict[str, tuple[str, int]],
) -> None:
    audit.require(path.exists(), f"effective v1.18 parameter export is missing: {path}")
    if not path.exists():
        return
    effective = parse_params(path)
    for name, (raw_value, parameter_type) in package.items():
        audit.require(name in effective, f"effective export is missing curated parameter {name}")
        if name in effective:
            audit.require(
                close(numeric(*effective[name]), numeric(raw_value, parameter_type)),
                f"effective {name} differs from the curated package",
            )
    for name, expected in airframe_values().items():
        audit.require(name in effective, f"effective export is missing airframe parameter {name}")
        if name in effective:
            audit.require(close(numeric(*effective[name]), expected), f"effective {name} differs from airframe 6100")
    audit.note(f"validated {len(package)} curated and {len(airframe_values())} airframe-owned effective values")


def inventory_text(
    legacy: dict[str, tuple[str, int]],
    package: dict[str, tuple[str, int]],
    metadata: dict[str, dict[str, object]],
) -> str:
    output = io.StringIO()
    writer = csv.writer(output, delimiter="\t", lineterminator="\n")
    writer.writerow(["legacy_name", "legacy_value", "target_name", "target_value", "disposition", "rationale"])
    covered_targets: set[str] = set()

    for name in sorted(legacy):
        old_value, _old_type = legacy[name]
        target = SUCCESSORS.get(name, name if name in metadata else "")
        target_value = package[target][0] if target in package else ""
        rationale = ""

        if name in SUCCESSORS:
            disposition = "translated_successor"
            rationale = {
                "EKF2_AID_MASK": "Translate legacy EV position+yaw and vision-height semantics to the v1.18 fusion bitmask.",
                "EKF2_HGT_MODE": "Translate legacy vision height selection to EKF2_HGT_REF.",
                "COM_RC_OVERRIDE": "Replace enable/disable semantics with the v1.18 speed-based manual override.",
                "COM_RC_STICK_OV": "No numeric percent-to-speed conversion exists; use the reviewed v1.18 threshold.",
            }[name]
        elif name in package:
            covered_targets.add(name)
            old_numeric = numeric(old_value, legacy[name][1])
            new_numeric = numeric(*package[name])
            if close(old_numeric, new_numeric):
                disposition = "migrate_exact"
                rationale = "Preserve the accepted August vehicle-specific value with unchanged v1.18 semantics."
            else:
                disposition = "bench_override"
                rationale = OVERRIDE_RATIONALE.get(name, "Use the reviewed v1.18 bench value instead of the legacy value.")
        elif name.startswith(("PWM_", "PWM_MAIN_", "PWM_AUX_", "CA_")):
            disposition = "airframe_owned"
            rationale = "Airframe 6100 owns geometry, function assignment, and output defaults; do not layer legacy output parameters over it."
        elif name.startswith("CAL_"):
            disposition = "fresh_calibration"
            rationale = "Do not import sensor IDs or calibration coefficients across a major firmware migration; calibrate and verify installed sensors on v1.18."
        elif name.startswith(("MC_", "MPC_")):
            disposition = "v118_default_retune_later"
            rationale = "Start from native v1.18 controller semantics; tune only after the bench and restrained-flight gates."
        elif name in metadata:
            disposition = "v118_default_not_imported"
            rationale = "The value is not vehicle identity required for LAB-02A; use the reset v1.18/airframe default."
        else:
            disposition = "removed_not_imported"
            rationale = "No exact v1.18 parameter exists and no reviewed successor is required for LAB-02A."

        if target in package:
            covered_targets.add(target)
        writer.writerow([name, old_value, target, target_value, disposition, rationale])

    for target in sorted(set(package) - covered_targets):
        writer.writerow(
            [
                "",
                "",
                target,
                package[target][0],
                "new_v118_bench_parameter",
                OVERRIDE_RATIONALE.get(target, "Explicit v1.18 vehicle or bench contract; no exact legacy source parameter."),
            ]
        )
    return output.getvalue()


def validate_inventory(
    audit: Audit,
    expected: str | None,
    legacy_count: int,
    package: dict[str, tuple[str, int]],
) -> None:
    audit.require(INVENTORY.exists(), f"missing generated inventory {INVENTORY}")
    if not INVENTORY.exists():
        return
    actual = INVENTORY.read_text(encoding="utf-8")
    if expected is not None:
        audit.require(actual == expected, "migration-inventory.tsv is stale; rerun with --write-inventory")
    rows = list(csv.DictReader(io.StringIO(actual), delimiter="\t"))
    audit.require(sum(bool(row["legacy_name"]) for row in rows) == legacy_count, "inventory does not cover every legacy row")
    targets = {row["target_name"] for row in rows if row["target_name"]}
    audit.require(set(package) <= targets, "inventory does not explain every curated target parameter")
    counts = Counter(row["disposition"] for row in rows)
    audit.note("migration dispositions: " + ", ".join(f"{key}={counts[key]}" for key in sorted(counts)))


def validate_agent(audit: Audit) -> None:
    pins: dict[str, str] = {}
    for line in PINS.read_text(encoding="utf-8").splitlines():
        if line and not line.startswith("#"):
            name, value = line.split("=", 1)
            pins[name] = value
    audit.require(pins == EXPECTED_PINS, "agent/pins.env differs from the reviewed Jazzy/XRCE/px4_msgs pins")

    dockerfile = (ROOT / "agent/Dockerfile").read_text(encoding="utf-8")
    for value in (
        EXPECTED_PINS["ROS_IMAGE"],
        EXPECTED_PINS["XRCE_AGENT_COMMIT"],
        EXPECTED_PINS["FASTCDR_COMMIT"],
        EXPECTED_PINS["FASTDDS_COMMIT"],
        EXPECTED_PINS["PX4_MSGS_COMMIT"],
    ):
        audit.require(value in dockerfile, f"Dockerfile does not pin {value}")
    for label in (
        "io.lecar.xrce-agent.commit",
        "io.lecar.fastcdr.commit",
        "io.lecar.fastdds.commit",
        "io.lecar.px4-msgs.commit",
        "io.lecar.authority",
    ):
        audit.require(label in dockerfile, f"Dockerfile does not record image label {label}")

    compose = (ROOT / "agent/compose.yaml").read_text(encoding="utf-8")
    audit.require("privileged:" not in compose, "compose must not enable privileged mode")
    audit.require("921600" in compose, "compose does not use the reviewed serial baud")
    audit.require(
        "network_mode: host" not in compose,
        "Agent profile must not expose the DDS domain through host networking",
    )
    audit.require(
        "internal: true" in compose and "fw02-shadow" in compose,
        "Agent and observer must share the private internal fw02-shadow network",
    )
    audit.require("read_only: true" in compose, "shadow observer root filesystem must be read-only")
    audit.require(
        "ROS_LOG_DIR: /tmp/ros-log" in compose
        and "/tmp:rw,noexec,nosuid,size=16m,mode=1777" in compose,
        "read-only observer must confine ROS logs to the bounded ephemeral /tmp mount",
    )
    audit.require(
        "user: ${HOST_UID:-1000}:${HOST_GID:-1000}" in compose,
        "shadow observer must write evidence as the invoking host user",
    )

    monitor_path = ROOT / "agent/shadow_monitor.py"
    monitor_source = monitor_path.read_text(encoding="utf-8")
    ast.parse(monitor_source, filename=str(monitor_path))
    for forbidden in ("create_publisher", "create_client", "create_service"):
        audit.require(forbidden not in monitor_source, f"shadow observer contains forbidden ROS mutation API {forbidden}")
    audit.require("create_subscription" in monitor_source, "shadow observer creates no subscriptions")

    topics = (REPO / "src/modules/uxrce_dds_client/dds_topics.yaml").read_text(encoding="utf-8")
    for topic in (
        "/fmu/out/battery_status",
        "/fmu/out/failsafe_flags",
        "/fmu/out/input_rc",
        "/fmu/out/sensor_combined",
        "/fmu/out/timesync_status",
        "/fmu/out/vehicle_attitude",
        "/fmu/out/vehicle_status",
    ):
        audit.require(topic in topics, f"required observer topic {topic} is absent from the firmware DDS map")


def validate_artifacts(audit: Audit, ci: bool) -> None:
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))
    audit.require(document.get("flash_authorized") is False, "manifest must explicitly deny flash authorization")
    audit.require(document.get("flight_authorized") is False, "manifest must explicitly deny flight authorization")
    audit.require(document["v118"]["source_commit"] == "2272d2d46e2b295b67e57b0f305ca8889a26fb03", "unexpected firmware source commit")

    locations = {
        "v118/px4_fmu-v4_am_tilted_hex.px4": REPO / "build/px4_fmu-v4_am_tilted_hex/px4_fmu-v4_am_tilted_hex.px4",
        "v118/px4_fmu-v4_am_tilted_hex.bin": REPO / "build/px4_fmu-v4_am_tilted_hex/px4_fmu-v4_am_tilted_hex.bin",
        "v118/am_fmuv4_lab02a.params": PACKAGE,
        "legacy-v110/px4_fmu-v4_default.px4": WORKSPACE / "refactor_campaign/rollback/legacy-v110-1326-source-equivalent/px4_fmu-v4_default.px4",
        "legacy-v110/aug11-final-711-params.params": LEGACY_DEFAULT,
    }
    listed: set[str] = set()
    ci_rebuilt_firmware = {
        "v118/px4_fmu-v4_am_tilted_hex.px4",
        "v118/px4_fmu-v4_am_tilted_hex.bin",
    }
    for line in HASHES.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        digest, name = line.split(maxsplit=1)
        name = name.lstrip("*")
        listed.add(name)
        audit.require(name in locations, f"checksum lists unknown artifact {name}")
        if name in locations:
            path = locations[name]
            local_only = name.startswith("legacy-v110/")
            audit.require(path.exists() or (ci and local_only), f"missing local artifact {path}")
            if path.exists():
                if ci and name in ci_rebuilt_firmware:
                    audit.note(
                        f"CI rebuilt {name} carries the PR Git revision; its run-specific hash is recorded separately"
                    )
                else:
                    audit.require(sha256(path) == digest, f"checksum mismatch for {name}")
    audit.require(listed == set(locations), "checksum manifest does not list the exact expected artifact set")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--legacy", type=Path, default=LEGACY_DEFAULT)
    parser.add_argument("--write-inventory", action="store_true")
    parser.add_argument(
        "--ci",
        action="store_true",
        help="validate the checked inventory without requiring workspace-local rollback files",
    )
    parser.add_argument(
        "--effective",
        type=Path,
        help="also validate a QGC full v1.18 parameter export against the curated package and airframe 6100",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    audit = Audit()
    try:
        package = parse_params(PACKAGE)
        metadata = load_metadata()
        validate_package(audit, package, metadata)
        if args.effective:
            validate_effective_export(audit, args.effective, package)
        if args.ci:
            audit.require(not args.write_inventory, "--ci and --write-inventory cannot be combined")
            validate_inventory(audit, None, 711, package)
        else:
            audit.require(args.legacy.exists(), f"legacy parameter snapshot is missing: {args.legacy}")
            if not args.legacy.exists():
                raise FileNotFoundError(args.legacy)
            audit.require(sha256(args.legacy) == LEGACY_SHA256, "legacy parameter snapshot SHA-256 is not the LAB-01B accepted file")
            legacy = parse_params(args.legacy)
            audit.require(len(legacy) == 711, f"expected 711 legacy parameters, found {len(legacy)}")
            expected_inventory = inventory_text(legacy, package, metadata)
            if args.write_inventory:
                INVENTORY.write_text(expected_inventory, encoding="utf-8")
                audit.note(f"wrote {INVENTORY}")
            validate_inventory(audit, expected_inventory, len(legacy), package)
        validate_agent(audit)
        validate_artifacts(audit, args.ci)
    except (FileNotFoundError, KeyError, OSError, ValueError, json.JSONDecodeError) as error:
        audit.errors.append(str(error))

    for note in audit.notes:
        print(f"NOTE: {note}")
    if audit.errors:
        for error in audit.errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"FW-02 audit failed with {len(audit.errors)} error(s)", file=sys.stderr)
        return 1
    print("PASS: FW-02 curated parameters, migration inventory, Agent pins, observer, and artifacts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
