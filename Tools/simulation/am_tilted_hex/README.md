# Aerial-manipulator SITL lifecycle and fault gates

This local simulation-only harness validates the nominal lifecycle of the
custom tilted-hex airframes in both PX4 SIH and Gazebo Harmonic. The nominal
scenario validates:

1. isolated startup and estimator output;
2. Arm command acknowledgement and observed armed state;
3. Takeoff command acknowledgement, airborne state, and a bounded hold;
4. native Land command acknowledgement and observed `AUTO.LAND`;
5. landed detection and PX4 automatic disarm; and
6. a readable ULog with the lifecycle topics and no logged dropouts.

Run both layers from the repository root using the pinned Python environment:

```sh
.venv/bin/python Tools/simulation/am_tilted_hex/run_lifecycle.py
```

Run one layer or show the Gazebo GUI:

```sh
.venv/bin/python Tools/simulation/am_tilted_hex/run_lifecycle.py --simulator sih
.venv/bin/python Tools/simulation/am_tilted_hex/run_lifecycle.py --simulator gazebo --gui
```

Run the SAFE fault suite, one exact scenario, or nominal plus faults:

```sh
.venv/bin/python Tools/simulation/am_tilted_hex/run_lifecycle.py --simulator sih --scenario safe-faults
.venv/bin/python Tools/simulation/am_tilted_hex/run_lifecycle.py --simulator gazebo --scenario offboard-loss
.venv/bin/python Tools/simulation/am_tilted_hex/run_lifecycle.py --simulator all --scenario all
```

Each scenario starts a fresh PX4 rootfs and produces its own summary and ULog.
The target is rebuilt only before the first requested scenario for each
simulator.

## SAFE scenario boundary

| Scenario | Automated boundary | Campaign coverage |
| --- | --- | --- |
| `arm-grounded` | Arm is accepted and observed, but the vehicle remains armed and landed inside a tight motion envelope until a distinct command | SAFE-01/03 PX4-side Arm/Takeoff separation |
| `in-air-disarm-denied` | Normal Disarm is rejected while airborne, and armed/airborne truth is preserved before native Land recovery | SAFE-01/04 truthful rejection behavior |
| `offboard-loss` | Stopping the only Offboard setpoint stream selects the default native Position fallback, remains armed/airborne, and never reacquires Offboard | SAFE-01/02 source-loss boundary |
| `gcs-override` | An explicit GCS Position-mode request supersedes active Offboard setpoints and is not silently reversed | SAFE-01 direct GCS takeover boundary |
| `land-from-offboard` | Native Land is accepted and observed while setpoints are still fresh; forwarding stops only after PX4 owns Land, which continues through touchdown and auto-disarm | SAFE-02/04 authority handoff |
| `post-land-disarm` | In isolated SITL, auto-disarm is disabled, PX4 remains landed and armed, and exactly one normal Disarm completes the terminal path | SAFE-04 optional continuation |

These are PX4 boundary tests, not a substitute for the ROS gateway, setpoint
arbiter, or activation gate. Lease generations, reset acknowledgement,
measurement-correlated target seeding, lost ACK handling, client/process
restart, estimator degradation, physical RC input, and physical kill require
their own runtime or staged hardware fixtures. The GCS scenario does not claim
to emulate a physical RC switch.

The Position-successor scenarios use a broad short-duration containment
envelope, not the nominal Hold envelope. This verifies that the mode handoff
does not immediately fly away, descend, or reacquire Offboard; it does not
qualify Position-control tuning. Exact drift metrics remain in each summary
for the later controller-tuning gate.

The default run builds each target first. `--skip-build` is available only
for local iteration with already-current build products. Results, PX4 output,
isolated root filesystems, ULogs, hashes, and machine-readable summaries are
written below `build/am_tilted_hex_lifecycle/` and remain ignored artifacts.

The harness fails instead of attaching when another PX4 process, MAVLink port
owner, or Gazebo world is active. It commands only UDP SITL and has no serial,
upload, flash, or hardware mode. Its isolated Gazebo server and clients are
explicitly pinned to `127.0.0.1`, avoiding dependence on the active Wi-Fi,
VPN, or CGNAT interface. Passing it does not authorize an embedded build,
hardware test, or flight.

## Continuous integration

`.github/workflows/am_tilted_hex_sitl.yml` runs nominal plus fault gates as two
independently visible jobs on every pull request into, and push to, the
personal fork's `release/1.18` branch:

- SIH uses the PX4 host dependencies without external simulation packages;
- Gazebo installs the official PX4 Gazebo Harmonic dependencies and runs
  headless.

Both jobs use a clean recursive checkout, run the helper tests, build their
target once, execute nominal plus all six fault scenarios from isolated
rootfs instances, and retain summaries, PX4 output, ULogs, and checksums for
14 days. The workflow is also manually dispatchable. It performs continuous
integration only; it does not deploy, package, flash, or contact hardware.
