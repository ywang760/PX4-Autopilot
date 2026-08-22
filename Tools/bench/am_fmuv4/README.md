# FW-02: first FMUv4 PX4 v1.18 bench package

This directory prepares, but does not authorize, the first props-removed
hardware gate for the aerial manipulator. It pairs the firmware produced by
`px4_fmu-v4_am_tilted_hex` with a deliberately small parameter package, an
immutable ROS 2 Jazzy/Micro XRCE-DDS Agent image, a subscribe-only traffic
observer, a defined TELEM1 cable, checksum-locked recovery inputs, and the
staged [`LAB-02A runbook`](lab02a-runbook.md).

No file here authorizes flashing, output testing, Arm, propeller installation,
or flight. Flashing is a separately announced LAB-02A transition. Output and
Arm are later, separate authorizations after the disarmed gate passes.

## Reviewed migration boundary

The accepted August v1.10 snapshot contains 711 parameters. The generated
v1.18 metadata contains 1,318 parameters, but only 491 names are shared. The
fresh package imports 100 reviewed values, not all 711 legacy values.

The complete one-row-per-legacy-parameter disposition is in
[`params/migration-inventory.tsv`](params/migration-inventory.tsv). Its current
summary is:

| Disposition | Count | Meaning |
| --- | ---: | --- |
| Exact accepted value retained | 74 | Vehicle-specific value has unchanged v1.18 semantics |
| Reviewed bench override | 6 | A safer/new transport or logging value intentionally differs |
| Translated successor | 4 | Old semantic intent maps to a renamed/redesigned v1.18 parameter |
| Airframe 6100 owns it | 48 | Geometry, function assignment, and output defaults come from the embedded airframe |
| Fresh sensor calibration | 48 | Sensor IDs and calibration coefficients are not copied across the major upgrade |
| Native v1.18 controller, tune later | 86 | Old MC/MPC gains are not treated as valid under the new controller/normalization |
| Native v1.18 default | 278 | Not required to identify the vehicle for this bench gate |
| Removed, no bench successor | 167 | Obsolete or irrelevant legacy parameter is not imported |
| New v1.18 bench parameter | 17 | DDS, absent-hardware, logging, and estimator settings with no exact legacy row |

The six `bench_override` rows are the effective differences from a same-name
copy. The important behavioral decisions are:

- `COM_DISARM_PRFLT=10`: unlike v1.10's disabled timeout, an armed vehicle that
  does not take off automatically disarms after ten seconds. This supports the
  bench but does not make Arm safe.
- `COM_OF_LOSS_T=1` and `COM_OBL_RC_ACT=0`: offboard loss waits one second and
  selects Position. The physical RC mode switch and kill switch remain direct
  PX4 inputs.
- `MAN_OVERRIDE_SPD=1`: v1.18 uses stick-speed override; the old 12-percent
  position threshold has no valid numeric conversion. Explicit RC mode
  switching remains the primary trained takeover action.
- `MAV_0_CONFIG=0` and `SER_TEL1_BAUD=921600`: TELEM1 is DDS only. USB is the
  configuration/recovery connection, not a required in-flight tether.
- `SDLOG_BACKEND=1` and `SDLOG_MODE=2`: the bench writes SD ULog from boot
  through shutdown and does not stream logs over MAVLink.

Other boundaries are equally intentional:

- Airframe 6100 supplies the six-output order, rotor axes, motor functions,
  900/1075/1950 us endpoints, and allocator configuration. The package contains
  no `CA_*` or `PWM_*` row.
- The accepted RC channel 1--5, 9, and 10 calibration and mappings are retained.
  Channel 9 high (`2064`) is Arm/ON; channel 10 high (`2064`) is kill engaged.
- Board mounting offsets and the installed 6S 10 Ah battery conversion are
  retained.
- The new estimator explicitly uses external-vision horizontal/vertical
  position and yaw (`EKF2_EV_CTRL=11`) with vision height. GPS, barometer,
  magnetometer, optical flow, and range fusion are disabled because those
  sensors are not installed in the accepted vehicle configuration.
- Legacy `CAL_*`, `MC_*`, and `MPC_*` values are excluded. Installed IMUs are
  freshly calibrated on v1.18; native controller defaults remain until a later
  tuning gate.
- Legacy `COM_KILL_DISARM=30` has no direct v1.18 equivalent and is not
  emulated. Kill lockdown, landed detection, and normal Disarm must be observed
  under the staged qualification.

## Companion pin set and authority

[`agent/pins.env`](agent/pins.env) locks:

- Ubuntu 24.04/ROS 2 Jazzy through an immutable official image digest;
- Micro XRCE-DDS Agent `v2.4.3` at commit
  `73622810d984349b80bbac0ef55fc0b694d62222`;
- the Agent's moving Fast-CDR/Fast-DDS dependency branches to the reviewed
  `v2.2.7` and `v2.14.6` commits; and
- `PX4/px4_msgs` `release/1.18` at commit
  `598c7aad7b2386f9406ebd2a2f841619fddc3c78`.

Those versions follow the PX4 v1.18 Jazzy compatibility table in
[`docs/en/middleware/uxrce_dds.md`](../../../docs/en/middleware/uxrce_dds.md).
The local image verification also compares every message used by the observer
byte-for-byte with this PX4 source tree.

The observer creates subscriptions only. It has no publisher, service client,
command helper, or serial device. The Agent and observer share an internal
Docker network with no host networking or published port, preventing unrelated
host or lab-network ROS 2 nodes from discovering this bench domain. It records
arrival rate, maximum inter-arrival gap, and staleness for nine FC output topics
and fails the baseline window if a required topic is absent or below a
deliberately loose quality floor. This is a transport observation profile, not
command authority and not a control-loop latency certification.

Build and verify it offline:

```sh
cd Tools/bench/am_fmuv4/agent
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)
docker compose --env-file pins.env -f compose.yaml build
./verify-image.sh
```

The first build compiles Fast DDS, the Agent, and all pinned ROS interfaces and
can take several minutes. Later builds use Docker cache.

## TELEM1 cable contract

The installed controller is Pixracer-class FMUv4. PX4's Pixracer documentation
defines TELEM1 as a six-pin JST-GH UART on `/dev/ttyS1`:

| Pixracer TELEM1 pin | Signal | Bench connection |
| ---: | --- | --- |
| 1 | +5 V | **Leave disconnected**; do not back-power either computer |
| 2 | FC TX, 3.3 V logic | USB-UART adapter RX |
| 3 | FC RX, 3.3 V logic | USB-UART adapter TX |
| 4 | CTS input, 3.3 V logic | Leave disconnected |
| 5 | RTS output, 3.3 V logic | Leave disconnected |
| 6 | Ground | USB-UART adapter ground |

Use a genuine 3.3 V TTL USB-UART adapter, not an RS-232 adapter. Identify the
JST orientation from the board/pinout rather than wire color. Because the first
configuration uses `UXRCE_DDS_FLCTRL=0`, only TX, RX, and ground are connected.
Use the adapter's stable `/dev/serial/by-id/...` name on the SNUC. The cable is
defined by FW-02 but still must be assembled or procured and continuity-checked
with the aircraft unpowered before LAB-02A DDS work.

The authoritative upstream pin table and serial mapping are in
[`docs/en/flight_controller/pixracer.md`](../../../docs/en/flight_controller/pixracer.md).

## Reproduce the offline audit

Build the firmware first, then run both audits:

```sh
make px4_fmu-v4_am_tilted_hex
python3 Tools/ci/am_fmuv4/audit.py \
  --build-dir build/px4_fmu-v4_am_tilted_hex
python3 Tools/bench/am_fmuv4/fw02_audit.py
```

`fw02_audit.py` validates all 100 package values against generated v1.18
metadata, rejects calibration/controller/output leakage, regenerates the full
711-row inventory, verifies immutable Agent pins and subscribe-only source,
and checks the exact v1.18 and local rollback hashes. Use
`--write-inventory` only when intentionally reviewing migration policy changes.
CI uses `--ci`, which validates the checked inventory without requiring the
workspace-local legacy recovery files.

After a lab import, validate a full QGC parameter export without hand-comparing
the package and airframe-owned values:

```sh
python3 Tools/bench/am_fmuv4/fw02_audit.py \
  --effective EVIDENCE_DIRECTORY/v118-effective.params
```

The exact lab build and recovery identity is in
[`artifacts/build-manifest.json`](artifacts/build-manifest.json) and
[`artifacts/SHA256SUMS`](artifacts/SHA256SUMS). The legacy image is explicitly a
source-equivalent recovery candidate, not the missing historically flashed
binary; restoring it also requires the matched 711-parameter export and a
separate rollback decision.

The checksum-locked v1.18 bytes were built from the merged FW-01 source commit
recorded in the manifest. FW-02 changes only bench tooling and documentation,
but PX4 embeds the current Git revision in every rebuild, so a CI image built at
the FW-02 commit is intentionally not byte-identical. Hosted CI writes
`ci-source-commit.txt` and `ci-SHA256SUMS` beside that run's firmware. Neither
the run-specific CI image nor its hash silently replaces the lab artifact in
`SHA256SUMS`.
