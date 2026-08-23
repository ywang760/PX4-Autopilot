# LAB-02A runbook: first PX4 v1.18 props-removed bench gate

## Objective and authority boundary

LAB-02A answers five questions in order:

1. Can the installed FMUv4 always be reached and recovered through direct USB?
2. Does the reviewed v1.18 image boot with the installed sensors, SD card, RC,
   airframe, and reviewed fresh parameter package?
3. Do raw battery voltage/current observations expose or constrain the known
   disagreement between QGC remaining percentage and an external tester?
4. Does dedicated TELEM1 DDS remain healthy while the FC logs and while both
   FC and SNUC resource margins are measured?
5. Does the link recover cleanly from a stopped Agent while the vehicle stays
   disarmed?

This document does not itself authorize a flash. At the lab, A0 ends at a
recorded go/no-go decision and the flight lead must explicitly authorize A1
before QGC writes firmware. Passing A1--A4 does not authorize outputs or Arm.
Any props-removed output test and any Arm test are later stages requiring their
own review and spoken authorization. No propeller installation or flight is in
LAB-02A.

Review `refactor_campaign/SAFETY_INCIDENTS.md` before the session. On this
vehicle, Arm and kill release are launch-capable commands. All six propellers
must remain removed and physically segregated for the entire visit even though
A0--A4 remain disarmed.

## Roles and universal aborts

Use a flight lead/QGC operator and an RC safety operator. Before applying
aircraft power:

- channel 9 is `964` (Arm switch OFF/Disarm);
- channel 10 is `2064` (kill engaged);
- the transmitter is on and controlled by the RC safety operator;
- the ROS 1 flight stack, autonomy nodes, and every command publisher are off;
- the six propellers are removed and labeled outside the work envelope; and
- the propulsion battery remains disconnected unless an installed battery or
  power-module observation explicitly requires it.

Abort the current stage on unexpected Arm, output motion, reboot, hard fault,
USB loss, parameter-reset ambiguity, sensor identity ambiguity, DDS type/error
spam, missing SD logging, resource exhaustion, a declining memory trend, or a
mode/RC mapping different from the LAB-01 record. Also stop any battery-powered
stage on damaged/puffed/hot cells, unexpected voltage collapse, or an
unexplained raw voltage/current discrepancy. Engage physical kill if needed,
obtain an authoritative Disarmed observation, remove propulsion power, and
preserve evidence. Do not improvise a parameter, wiring, or firmware fix inside
the same run.

## Before leaving for the lab

- [ ] Review the FW-02 PR, this runbook, the FW-02 README, and the safety
  incident register.
- [ ] Obtain or assemble the defined three-wire TELEM1-to-3.3-V-TTL USB-UART
  cable. Pins 1, 4, and 5 remain unconnected.
- [ ] Build and verify `am-px4-jazzy-xrce:fw02` on the SNUC or export/import
  that exact locally verified image. Do not build from moving tags at the lab.
- [ ] Copy the exact v1.18 `.px4` from
  `refactor_campaign/lab/LAB-02/artifacts/fw02-candidate-2272d2d46e/`, the
  reviewed `.params`, SHA256 manifest, and the complete source-equivalent v1.10
  recovery directory to the recovery workstation. Keep a second offline copy;
  do not substitute a later file from the mutable PX4 `build/` directory.
- [ ] Confirm direct USB cable reach from the FC to the recovery workstation.
  USB is required for this bench/recovery session; it is not a flight tether
  architecture decision.
- [ ] Bring the identified 6S battery, an independent trusted pack/per-cell
  voltmeter or tester, and—if current accuracy is to be qualified—a trusted
  current/coulomb reference or charger-capacity record. Percentage alone is
  not a calibration reference.
- [ ] Prepare an evidence directory named with UTC time and a unique run ID.

## A0 — offline identity and direct-USB recovery readiness

Keep aircraft power off while connecting or continuity-checking cables.

1. Run the complete offline audit from the PX4 repository:

   ```sh
   python3 Tools/bench/am_fmuv4/fw02_audit.py
   cd Tools/bench/am_fmuv4/agent
   ./verify-image.sh
   ```

2. Copy `artifacts/build-manifest.json`, `artifacts/SHA256SUMS`, both audit
   outputs, and the current git commit/status into the evidence directory.
3. Continuity-check the unpowered TELEM1 cable end-to-end: FC TX to adapter RX,
   FC RX to adapter TX, and ground to ground. Prove +5 V, CTS, and RTS are not
   connected. Record the adapter identity and its `/dev/serial/by-id/...` path.
4. Connect the FC's native USB directly to the recovery workstation. Power
   avionics only. Confirm QGC receives heartbeat, parameters, and shell access.
5. Before any write, export the still-installed v1.10 parameters into the
   evidence directory and record `ver all`, board/bootloader identity, QGC
   version, USB device identity, Disarmed state, and kill state.
6. In QGC, confirm that **Custom firmware file** can select the checksum-locked
   v1.18 `.px4`. Do not click the final flash action yet. Confirm the
   source-equivalent v1.10 `.px4` is also locally selectable.

**A0 gate:** QGC/direct USB is reliable, both files and parameter exports are
local and checksum-valid, the vehicle is Disarmed with props removed, and the
recovery operator can state the rollback sequence. Otherwise stop without
flashing.

## A1 — separately authorized v1.18 flash and clean configuration

The flight lead must announce `AUTHORIZE A1 V1.18 FLASH` and record the time.
No other stage authorization implies this one.

1. Stop all SNUC ROS/DDS/Agent containers. QGC over direct FC USB is the only
   active interface.
2. Flash the checksum-locked `px4_fmu-v4_am_tilted_hex.px4` through QGC's
   custom-firmware path. Do not use the raw `.bin`.
3. If flash or boot fails, stop. Use QGC/direct USB recovery. Do not silently
   switch to another PX4 build. Flashing the source-equivalent v1.10 recovery
   image is a separately recorded rollback decision and must be followed by
   its matching 711-parameter import.
4. After v1.18 boots, record `ver all` and confirm the expected source hash.
5. Reset all parameters to v1.18 firmware defaults and reboot.
6. Select airframe **6100 AM Tilted Hex** and reboot. Do not manually recreate
   its geometry or output mapping.
7. Import only `am_fmuv4_lab02a.params` and reboot again.
8. Export the complete effective v1.18 parameters. Confirm:

   - `SYS_AUTOSTART=6100`, `MAV_TYPE=13`, `CA_ROTOR_COUNT=6`;
   - output functions 1--6 are Motor 1--6;
   - outputs 1--6 show disarmed/min/max `900/1075/1950`;
   - TELEM1 has MAVLink disabled, DDS enabled, and 921600 baud;
   - RC maps are channels 1/2/3/4, mode 5, Arm 9, kill 10; and
   - every value in the reviewed package matches the import. The current count
     is recorded as evidence, not treated as a fixed target.

   Run the machine check against that full export:

   ```sh
   python3 Tools/bench/am_fmuv4/fw02_audit.py \
     --effective EVIDENCE_DIRECTORY/v118-effective.params
   ```

Do not change a mismatch by hand. Preserve the export and stop for offline
review.

## A2 — installed sensors, SD, RC, and fresh calibration

Remain Disarmed. Do not run actuator tests and do not authorize Arm.

1. Record QGC Sensors/Health and the PX4 shell output from:

   ```text
   sensors status
   ekf2 status
   logger status
   hardfault_log check
   dmesg
   ```

2. Confirm the expected two IMU paths are present and stable. Confirm absent
   GPS, barometer, magnetometer, optical flow, and range devices are not
   required by parameters. An unexpected sensor is an inventory mismatch, not
   a reason to enable it ad hoc.
3. Perform fresh v1.18 gyro and accelerometer calibration with the airframe
   stationary and props removed. Save the post-calibration parameter export.
   Do not import legacy `CAL_*` coefficients.
4. Verify the board orientation and the LAB-01 tilt directions by hand-moving
   the unpowered-output vehicle while observing QGC attitude. Do not arm.
5. Move RC channels through their LAB-01 positions and record live values:
   channel 5 `964/1514/2064`, channel 9 OFF at `964`, and channel 10 kill
   engaged at `2064`. Verify roll/pitch/yaw/throttle direction and neutral.
   Do not invoke QGC RC calibration unless the imported calibration is shown
   invalid and reviewed separately.
6. Verify the SD card mounts and `logger status` reports boot-to-shutdown SD
   logging. Power-cycle once and prove a new ULog was created and closed.
7. Perform the battery/power-module observation because prior operation showed
   QGC reporting less remaining capacity than an external tester:

   - begin from the offline finding that the legacy current signal was only
     `0.117--0.410 A` median during 7.024 kg airborne operation and therefore
     did not measure total aircraft current; do not accept a plausible-looking
     idle number as resolution of that finding;
   - identify the battery and record chemistry, nominal capacity, cycle/history
     information if known, and physical condition;
   - trace and record whether the propulsion and SNUC battery branches actually
     pass through the sensor feeding the Pixracer `POWER/CSen` input, without
     disturbing hidden wiring during this run;
   - before connection, record pack voltage and every cell voltage with the
     independent instrument;
   - after the documented avionics/SNUC load stabilizes, record timestamp,
     SNUC power state, external pack voltage, PX4/QGC voltage, current,
     remaining percentage, and discharged capacity;
   - export `BAT1_V_DIV`, `BAT1_A_PER_V`, `BAT_V_OFFS_CURR`,
     `BAT1_I_OVERWRITE`, `BAT1_R_INTERNAL`, `BAT1_I_FILT`, `BAT1_V_FILT`,
     `BAT1_CAPACITY`, `BAT1_N_CELLS`, `BAT1_V_CHARGED`,
     `BAT1_V_EMPTY`, `BAT_LOW_THR`, `BAT_CRIT_THR`, `BAT_EMERGEN_THR`,
     `COM_LOW_BAT_ACT`, `COM_FLTT_LOW_ACT`, and `COM_ARM_BAT_MIN`;
   - record `BAT1_C_MULT` only to establish that the SMBus-only multiplier is
     inactive for the selected analog source; do not use it as a current fix;
   - if a trusted current/coulomb reference is available, compare it under the
     same steady load. Otherwise mark current scale and remaining percentage
     unqualified rather than inferring current accuracy from voltage; and
   - do not change battery parameters in this run or force QGC percentage to
     match a voltage-based tester percentage.

**A2 gate:** installed sensor identity is understood, fresh IMU calibration is
saved, orientation and RC mappings match, SD logging survives reboot, and the
battery comparison is captured with raw quantities and load context. Any
unexplained voltage mismatch is a stop; current/remaining accuracy may remain
explicitly unqualified for this disarmed gate but must be resolved before a
props-on gate. No output or Arm has occurred.

## A3 — disarmed DDS and resource baseline

Remain Disarmed with channel 9 OFF and kill engaged. The observer is
subscribe-only.

1. With aircraft power off, connect the verified TELEM1 adapter. Reapply
   avionics power and preserve QGC/direct USB recovery.
2. On the SNUC, set the stable adapter path and create the evidence directory:

   ```sh
   cd Tools/bench/am_fmuv4/agent
   export HOST_UID=$(id -u)
   export HOST_GID=$(id -g)
   export XRCE_SERIAL_DEVICE=/dev/serial/by-id/REPLACE_WITH_RECORDED_ID
   export SHADOW_OUTPUT_DIR="$PWD/evidence"
   export SHADOW_DURATION=600
   mkdir -p "$SHADOW_OUTPUT_DIR"
   docker compose --env-file pins.env -f compose.yaml up -d xrce-agent shadow-observer
   ```

3. Confirm Agent logs show a client session without repeated create/delete or
   type errors. From the isolated observer container, confirm the output topics:

   ```sh
   docker compose --env-file pins.env -f compose.yaml exec shadow-observer \
     ros2 topic list
   ```

   Do not attach another container to `fw02-shadow` and do not run any
   publisher, service, action, topic echo with mutation, or ROS 1 flight stack.
4. Capture these FC shell snapshots before the Agent, after connection, at
   minutes 2, 5, and 10, and after stopping it:

   ```text
   uxrce_dds_client status
   top once
   free
   work_queue status
   uorb top -1
   logger status
   perf
   ```

   `uxrce_dds_client status` must report serial, connected, converged time sync,
   and payload TX/RX rates. `top once` is the runtime stack high-water evidence
   that the offline linker report cannot provide.
5. Capture SNUC `docker stats --no-stream`, `free -h`, `vmstat 1 10`, Agent
   logs, and the image ID. Preserve `dds-shadow.json` after the observer exits.

The baseline passes only if the observer passes all required topics, no FC
task has less than both 20 percent and 512 bytes stack remaining, the uXRCE
task retains at least 25 percent of its 9000-byte declaration, heap does not
decline after warm-up, CPU has clear non-saturating margin, logging stays
active, and no restart/allocation/hard-fault/type error occurs. Treat those as
minimum screening margins, not flight qualification.

## A4 — disarmed Agent-dropout recovery

This is a software-side Agent stop, not a cable pull. Stay Disarmed and keep
QGC/direct USB connected.

1. Start a separate observer evidence window and confirm the baseline is
   healthy.
2. Record the time, stop only `xrce-agent` for ten seconds, and observe PX4/QGC:

   ```sh
   docker compose --env-file pins.env -f compose.yaml stop xrce-agent
   ```

3. Confirm the FC stays booted, Disarmed, and responsive over USB. Record
   `uxrce_dds_client status`, `top once`, `free`, and events during the gap.
4. Restart only the exact pinned Agent and record time to session recovery:

   ```sh
   docker compose --env-file pins.env -f compose.yaml up -d xrce-agent
   ```

5. Run a fresh 120-second observer window after reconnection. The gap window is
   expected to fail continuity; the post-recovery window must pass. Confirm a
   ULog spans the event and closes cleanly at shutdown.
6. Stop and archive the containers without removing the built image:

   ```sh
   docker compose --env-file pins.env -f compose.yaml logs --no-color xrce-agent \
     > evidence/xrce-agent.log
   docker compose --env-file pins.env -f compose.yaml down
   ```

**A4 gate:** a stopped Agent does not reset, arm, change mode, or exhaust the
FC, and the exact Agent reconnects with a clean post-recovery traffic window.

## Explicit stop point

End LAB-02A after A4. Export the final v1.18 parameters, close/copy the ULogs,
hash the whole evidence directory, shut down safely, and review results
offline. Do not proceed directly to motor output testing or Arm.

If A0--A4 pass, prepare a separate LAB-02B proposal for:

1. target-oriented review of output failure state, Commander/failsafe,
   land/disarm, and battery parameters, including the unresolved QGC battery
   estimate; then
2. props-removed output 1--6 isolation/order/direction/endpoints; then
3. a separately authorized Arm-only idle interval, including the ten-second
   preflight auto-disarm and kill behavior.

Those stages must reuse the LAB-01 physical map and explicitly regress SI-001.
They remain non-flight tests and are not authorized by this runbook. Native
MC/MPC parameters and mocap-to-EKF timing/quality require independent review
and test before restrained or free flight. Near-term flight qualification is
mocap-only; camera/policy/external-perception flight is out of scope.
