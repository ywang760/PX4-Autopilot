# Aerial-manipulator SITL lifecycle gate

This local simulation-only harness validates the nominal lifecycle of the
custom tilted-hex airframes in both PX4 SIH and Gazebo Harmonic:

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

The default run builds each target first. `--skip-build` is available only
for local iteration with already-current build products. Results, PX4 output,
isolated root filesystems, ULogs, hashes, and machine-readable summaries are
written below `build/am_tilted_hex_lifecycle/` and remain ignored artifacts.

The harness fails instead of attaching when another PX4 process, MAVLink port
owner, or Gazebo world is active. It commands only UDP SITL and has no serial,
upload, flash, or hardware mode. Passing it does not authorize an embedded
build, hardware test, or flight.

## Continuous integration

`.github/workflows/am_tilted_hex_sitl.yml` runs the same gate as two required
and independently visible jobs on every pull request into, and push to, the
personal fork's `release/1.18` branch:

- SIH uses the PX4 host dependencies without external simulation packages;
- Gazebo installs the official PX4 Gazebo Harmonic dependencies and runs
  headless.

Both jobs use a clean recursive checkout, run the helper tests, build their
target, execute the lifecycle, and retain summaries, PX4 output, ULogs, and
checksums for 14 days. The workflow is also manually dispatchable. It performs
continuous integration only; it does not deploy, package, flash, or contact
hardware.
