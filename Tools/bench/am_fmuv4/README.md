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
current fresh package imports 100 reviewed values, not all 711 legacy values.
The number 100 is the outcome of the present semantic selection, not a target
size, quota, or stable interface. Package identity and rationale matter; its
count may change when a parameter decision changes.

The complete one-row-per-legacy-parameter disposition is in
[`params/migration-inventory.tsv`](params/migration-inventory.tsv). Its current
summary is:

| Disposition | Count | Meaning |
| --- | ---: | --- |
| Exact accepted value retained | 73 | Vehicle-specific value has unchanged v1.18 semantics |
| Same value, changed semantics | 1 | `BAT1_R_INTERNAL=-1` changes from throttle-based to measured-current-based sag estimation and remains unqualified |
| Reviewed bench override | 6 | A safer/new transport or logging value intentionally differs |
| Translated successor | 4 | Old semantic intent maps to a renamed/redesigned v1.18 parameter |
| Airframe 6100 owns it | 48 | Geometry, function assignment, and output defaults come from the embedded airframe |
| Fresh sensor calibration | 48 | Sensor IDs and calibration coefficients are not copied across the major upgrade |
| Native v1.18 controller, tune later | 86 | Old MC/MPC gains are not treated as valid under the new controller/normalization |
| Native v1.18 default | 278 | Not required to identify the vehicle for this bench gate |
| Removed, no bench successor | 167 | Obsolete or irrelevant legacy parameter is not imported |
| New v1.18 bench parameter | 17 | DDS, absent-hardware, logging, and estimator settings with no exact legacy row |

This is a complete **legacy-oriented migration inventory**, not a claim that
every v1.18 parameter has been individually accepted. The generator applies a
small explicit successor table and broad ownership rules after the package has
been manually selected. In particular, a remaining same-name parameter is
currently labeled `v118_default_not_imported`; that label means "do not
override the v1.18 default for LAB-02A," not "unused" or "semantically audited
for flight."

## Target-side review boundary

The package is an override layer, not the complete effective PX4
configuration. After reset, all 1,318 compiled parameters exist; airframe 6100,
the package, and fresh calibration then override subsets of those defaults.
The current target namespace divides as follows:

| Current treatment | Target parameters |
| --- | ---: |
| Explicit LAB-02A package | 100 |
| Explicit airframe 6100 values outside the package | 70 |
| Written by fresh calibration rather than legacy import | 40 |
| Native MC/MPC values explicitly deferred for later tuning | 69 |
| Legacy-named parameters left at native v1.18 defaults | 277 |
| `PWM_MAIN_FAIL1`--`PWM_MAIN_FAIL6` left at native v1.18 defaults | 6 |
| New v1.18-only parameters left at native defaults | 756 |
| **Total** | **1,318** |

Many parameters outside the package are active. For example, native MC/MPC
gains will control the aircraft, fresh `CAL_*` values will be written during
calibration, and Commander and land-detector defaults still define behavior.
Others belong to inactive or absent hardware and vehicle classes. Absence from
the package only means that LAB-02A does not override the compiled/airframe
default.

Before any output/Arm gate, perform a target-oriented review of output failure
state, Commander/failsafe, land/disarm, and battery/power parameters. Before
restrained or free flight, additionally review and independently test native
MC/MPC control parameters and the mocap-to-EKF timing/quality contract. A
review candidate does not automatically become an imported value: retaining a
native v1.18 default can be the reviewed result. The six `PWM_MAIN_FAIL*`
parameters are called out separately because the broad legacy `PWM_*`
classification labels them airframe-owned, while airframe 6100 does not
currently set them explicitly.

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
- In the near-term campaign, "external vision" means the mocap pose delivered
  to PX4. Camera/policy/external-perception flight is out of scope; no setting
  here qualifies that path.
- Legacy `CAL_*`, `MC_*`, and `MPC_*` values are excluded. Installed IMUs are
  freshly calibrated on v1.18; native controller defaults remain until a later
  tuning gate.
- Legacy `COM_KILL_DISARM=30` has no direct v1.18 equivalent and is not
  emulated. Kill lockdown, landed detection, and normal Disarm must be observed
  under the staged qualification.

## Battery evidence and qualification boundary

The current aircraft has previously shown a lower QGC remaining percentage
than an external battery tester. Treat battery remaining as unqualified until
the discrepancy is explained. QGC percentage and a simple voltage-based tester
percentage are both model estimates; neither is accepted as ground truth by
itself.

The offline audit of all 15 accepted August flight ULogs and both LAB-01B
ULogs establishes that the legacy current channel is not total aircraft
current: per-log airborne medians are only `0.117--0.410 A`, 14--38 percent of
airborne samples are exactly zero, and current has essentially no collective
correlation while pack voltage does. The integrated counter advances only
`3.41--10.17 mAh` per accepted flight while displayed remaining moves by up to
29.74 percentage points. The reproducible evidence and exact hashes are in
`refactor_campaign/baseline/battery-telemetry-audit.md`. This does not identify
whether the cause is sensor topology, wiring, module compatibility, scale, or
offset; no scale change is justified yet.

PX4 v1.18's voltage-sag/internal-resistance model also depends on measured
current, so the firmware upgrade cannot make remaining/current valid without
physical current-path evidence. The imported `BAT1_V_CHARGED=4.2` is retained
only as the legacy starting condition for the disarmed diagnostic gate. Both
v1.10 and v1.18 describe the loaded/full estimator reference as normally below
the 4.2 V nominal maximum; it is an explicit `FW-03` review/test candidate, not
an accepted flight value.

LAB-02A records raw pack and per-cell voltage before connection, externally
measured voltage under the documented avionics load, PX4/QGC voltage, current,
remaining percentage, discharged capacity, battery identity, and SNUC power
state. Review at least `BAT1_V_DIV`, `BAT1_A_PER_V`, `BAT_V_OFFS_CURR`,
`BAT1_I_OVERWRITE`, `BAT1_R_INTERNAL`, voltage/current filter settings,
capacity/cell/charged/empty settings, and low/critical/emergency thresholds and
Commander actions (`COM_LOW_BAT_ACT`, `COM_FLTT_LOW_ACT`, and
`COM_ARM_BAT_MIN`). `BAT1_C_MULT` is an SMBus-only setting and is not an
analog-current correction while `BAT1_SOURCE=0`; verify it remains inactive.
Do not tune a scale merely to make two percentage displays agree. Current-scale
and remaining-capacity qualification requires a trusted current/coulomb
reference or controlled charger-returned capacity evidence.

Until that review passes, QGC remaining percentage is advisory rather than a
sole flight-abort signal. Any future props-on gate must use a separately
approved conservative raw-voltage/per-cell limit, flight-time/capacity limit,
and physical battery inspection in addition to QGC reporting.

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

`fw02_audit.py` validates every current package value against generated v1.18
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

The checksum-locked v1.18 bytes live outside the mutable PX4 build directory at
`refactor_campaign/lab/LAB-02/artifacts/fw02-candidate-2272d2d46e/`. This
separation is deliberate: a normal `make` overwrites `build/` and embeds build
identity, so an old checksum is not useful unless the exact bytes it names are
also retained. The local audit validates that immutable copy; CI validates a
fresh same-source build and publishes separate run-specific hashes.

The checksum-locked v1.18 bytes were built from the merged FW-01 source commit
recorded in the manifest. FW-02 changes only bench tooling and documentation,
but PX4 embeds the current Git revision in every rebuild, so a CI image built at
the FW-02 commit is intentionally not byte-identical. Hosted CI writes
`ci-source-commit.txt` and `ci-SHA256SUMS` beside that run's firmware. Neither
the run-specific CI image nor its hash silently replaces the lab artifact in
`SHA256SUMS`.
