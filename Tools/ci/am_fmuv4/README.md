# Aerial Manipulator FMUv4 firmware audit

This directory covers the offline `FW-01` feasibility gate for the installed
Pixracer-class FMUv4. It does not authorize flashing, powering actuators, or
flight.

## Target and result

The embedded target is:

```sh
make px4_fmu-v4_am_tilted_hex
```

The target inherits `px4/fmu-v4/default`, enables the standard PX4
uXRCE-DDS client and its unmodified upstream topic file, and explicitly
disables only unrelated vehicle classes and absent peripherals. The build uses
hardware airframe `6100_am_tilted_hex`.

Measured at PX4 commit `60bf3e99de992c52a3a2bc32683bedd560c38a53`
with Ubuntu 24.04 and `arm-none-eabi-gcc 13.2.1`:

| Image | Flash (`text + data`) | FMUv4 flash | Static SRAM (`data + bss`) | FMUv4 SRAM |
| --- | ---: | ---: | ---: | ---: |
| Upstream `px4_fmu-v4_default` plus the new ROMFS airframe | 1,711,732 B | 82.26% | 28,560 B | 14.53% |
| `px4_fmu-v4_am_tilted_hex` | 1,507,932 B | 72.47% | 27,616 B | 14.05% |

The AM image saves 203,800 bytes of flash relative to the same-commit default
and leaves 572,836 bytes free in the linker's 2,032 KiB flash region. GNU
`size` static SRAM does not include heap, dynamically created tasks, runtime
message buffers, or every work-queue stack. The offline result therefore
passes the image-fit gate but cannot establish runtime RAM margin.

The CI audit fails above 80% flash or 20% static SRAM. Those are campaign
review thresholds, not claims that the remaining memory is sufficient for
flight.

## Retained flight and recovery surface

The audit requires all of these capabilities to remain enabled:

- Commander, Events, Navigator, Flight Mode Manager, manual control, RC input,
  land detection, and battery status;
- EKF2, installed ICM20602/MPU9250 sensors, the ICM20608G board-revision
  fallback, barometers, magnetometers, GPS, board ADC, and gyro calibration;
- multicopter position, attitude, rate, and hover-thrust control plus Control
  Allocator;
- PWM output, safety button, tone alarm, SD-card Dataman and logging;
- MAVLink, USB CDC/QGC access, uXRCE-DDS, load monitoring, `top`, uORB and work
  queue inspection; and
- actuator test, hard-fault log, parameter, bootloader-update, version, and
  reboot recovery commands.

No safety, flight, estimator, logging, observability, or recovery component is
removed to make the image fit.

## Explicit differences from upstream FMUv4

The only added Kconfig root is `CONFIG_MODULES_UXRCE_DDS_CLIENT`. The explicit
removals are:

| Area | Removed because it is not installed or not this vehicle class |
| --- | --- |
| Other vehicles | fixed-wing attitude/autotune/mode/lateral-longitudinal/rate controllers, VTOL attitude control, and UUV attitude/position control |
| Alternate estimators/tools | local position estimator and multicopter autotune; EKF2 and the flown manual gain workflow remain |
| Air-data and ranging | airspeed selector, differential-pressure drivers, distance-sensor drivers, IR-Lock, and landing-target estimator |
| Camera/gimbal | FC camera capture, camera trigger, camera feedback, and gimbal module; the manipulator camera remains a SNUC peripheral |
| Absent hardware | Septentrio-specific GNSS, ADIS16448 and ICM20948 IMUs, PCA9685 output, and PWM input |
| Test fixture | fake GPS example |

Disabling the two common sensor groups also disables their selected child
drivers. `audit.py` reports the complete resolved Kconfig diff so those
dependency removals cannot be hidden by the short table.

## Embedded airframe contract

Airframe 6100 carries the flight-proven physical mapping:

| PX4 output | Physical motor |
| ---: | --- |
| 1 | mid-right |
| 2 | mid-left |
| 3 | front-left |
| 4 | rear-right |
| 5 | front-right |
| 6 | rear-left |

It uses the reviewed normalized tilted axes, native v1.18 pseudo-inverse plus
clipping (`CA_METHOD=0`), and `MAV_TYPE=13`. It also preserves the accepted
v1.10 ESC pulse contract as defaults: disarmed 900 us, minimum 1075 us,
maximum 1950 us, and 400 Hz through the upstream timer default. These remain
props-removed verification items, not assumed flight equivalence.

Airframe commands use `param set-default` only. Saved parameters have higher
authority. In particular, the accepted v1.10 snapshot contains
`MAV_0_CONFIG=101`; importing it unchanged would conflict with the new TELEM1
DDS assignment. A later flash runbook must start from a reviewed v1.18
parameter set and verify the effective values rather than relying on airframe
defaults to overwrite migrated values.

## Serial/DDS budget

The initial physical assignment is deliberately non-shared:

- TELEM1 (`/dev/ttyS1`) at 921600 baud: standard uXRCE-DDS client only;
- USB CDC: MAVLink/QGC console, configuration, and recovery; and
- TELEM2: reserved for an independent future MAVLink link if required.

The airframe sets `MAV_0_CONFIG=0`, `UXRCE_DDS_CFG=101`, and
`SER_TEL1_BAUD=921600` as defaults. MAVLink and uXRCE-DDS must never be placed
on the same byte stream.

The audit reads serialized sizes from the generated uCDR headers. Its expected
steady-state profiles are:

| Direction/profile | Serialized payload | With 25% protocol reserve | 921600-baud 8-N-1 capacity |
| --- | ---: | ---: | ---: |
| FC to SNUC: core local-state/health/RC topics | 41,561 B/s | 51,951 B/s | 92,160 B/s |
| FC to SNUC: core plus optional GPS/global topics | 53,861 B/s | 67,326 B/s | 92,160 B/s |
| SNUC to FC: 100 Hz mocap odometry and rate setpoint, 20 Hz Offboard heartbeat, 5 Hz command allowance | 15,280 B/s | 19,100 B/s | 92,160 B/s |

UART is full duplex, so inbound and outbound use separate directions. The
estimate assumes the v1.18 default 200 Hz `sensor_combined` source. It excludes
sporadic event/ack traffic and cannot predict XRCE retries, scheduling delay,
or actual framing/aggregation. A props-removed bench must measure both wire
rate and dropped/stale topics under logging and command load. Do not customize
the upstream topic file unless that measurement fails.

## Task-stack audit and remaining runtime gate

The largest declared built-in entry stack is the upstream uXRCE-DDS client at
9000 bytes. Other relevant declarations include Control Allocator at 3000,
Logger's entry task at 2500, and the default entry stack at 2048. The logger
worker configuration is 3700 bytes. Selected work-queue configurations include
6000-byte INS and VTE queues, 3150-byte rate control, 2392-byte SPI, 2240-byte
navigation/controllers, 2300-byte high-priority, 2400-byte low-priority, and
1900-byte I2C queues.

These are configured maxima, not simultaneous measured consumption. Before
any flight gate, a props-removed hardware session must record:

- successful boot, SD logging, QGC/USB recovery, RC, estimator, and all
  installed sensor status;
- `free`, `top`, work-queue status, `uxrce_dds_client status`, and task stack
  high-water evidence at idle and under representative DDS/logging traffic;
- uXRCE connection recovery after Agent restart and after serial disconnect;
- actual TX/RX rates, message age, loss/dropout, and CPU load; and
- all six isolated actuator outputs, pulse endpoints, prompt stop, Disarm,
  kill, and power-cycle rollback with props removed.

Poor runtime margin, serial loss, missing recovery access, missing sensors, or
the need to remove a retained safety/observability module rejects the FMUv4
path. Offline `FW-01` does not waive that decision gate.

## Reproduce the audit

```sh
make px4_fmu-v4_default
make px4_fmu-v4_am_tilted_hex
python3 Tools/ci/am_fmuv4/audit.py \
  --build-dir build/px4_fmu-v4_am_tilted_hex \
  --baseline-build-dir build/px4_fmu-v4_default \
  --report build/px4_fmu-v4_am_tilted_hex/fw01-audit.json
```

The JSON contains the exact resolved Kconfig diff, memory values, serialized
topic budget, stack declarations, and every pass/fail check. CI uploads the
JSON, firmware package, board configurations, and linker maps for review.
