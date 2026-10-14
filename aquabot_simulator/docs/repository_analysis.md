# Phase 0: repository analysis and Phase 1 plan

Analysis date: October 14, 2026. Initially reviewed commit `dc4cc68`; refreshed
against `origin/main` at `2460c6a` before committing the simulator. The Git
working tree was clean before implementation. No `AGENTS.md` instructions were
found. Actual source paths
start with `boat_control/`, not the root-level `core/` paths in the specification.
SHA-256 hashes of all 38 existing non-Git files were captured before edits for
the initial preservation check, which passed. The later user-authorized branch
sync imports upstream changes only. Simulator development continues to be
confined to `aquabot_simulator/`; existing source is read-only reference material.

## Existing control architecture

`boat_control/core/hardware/sensors/ublox_f9p.py` opens a GPS UART and handles
NAV-PVT position/velocity and NAV-RELPOSNED heading. It publishes GPS at 2 Hz.
The default heading source is dual-antenna relative position with a +90 degree
mount offset. Although the `motion` parameter value is still accepted, the
NAV-PVT motion-heading assignment is commented out in the current source. In
motion mode, relative heading handling also returns early, leaving heading
unset and preventing GPS publication. Future sensor integration must reflect
the currently implemented relative-position heading behavior.

`boat_control/core/vehicle_supervisor.py` owns arming and flight mode, exposes
`arm_disarm` and `flight_mode_change` services, and publishes supervisor state
at 20 Hz. It starts disarmed in HOLD and returns to HOLD on disarming.

`boat_control/core/mission/mission_manager.py` accepts mission items through
`upload_mission`, requests GUIDED after arming, and publishes the current goal
at 100 Hz while active. Waypoint x/y are latitude/longitude scaled by 1e7. It
uses GPS position and a hardcoded 2 m acceptance radius to advance intermediate
items. The final item remains the goal: the code does not mark completion,
disarm, enter HOLD, or publish a stop command on final arrival.

`boat_control/core/vehicle_controller.py` receives GPS, goal, manual input,
and supervisor state. In armed GUIDED it calls `ThrusterPIDControl.calc_control`
at 100 Hz. Distance is Haversine metres, heading is a local geographic bearing
clockwise from north, and heading error wraps to [-180, 180). It publishes
normalized motor commands after a 0.01-per-callback slew limit. Armed MANUAL
maps z to left and x to right. HOLD and disarmed cases publish no new command.

`boat_control/core/hardware/low_level_motor_ctrl.py` is the physical interface.
It opens an Arduino serial connection, scales commands by 0.75, maps to signed
integers [-128, 127], and writes ASCII pairs. It must never run in simulation.
The simulator neither imports it nor starts the existing launch file.

```text
GPS -> mission manager (arrival/goal selection) -> goal waypoint
GPS + goal + supervisor state -> vehicle controller -> waypoint PID
PID -> mixer -> slew limiter -> /vehicle/motor_power
physical path: motor power -> UART driver -> thrusters
new Phase 1 path: explicit Python commands -> motor lag -> thrust -> dynamics
later simulation path: motor power -> adapter -> dynamics -> simulated GPS
```

## Verified interface usage

These types and fields are verified from consumers/producers. The `boat_iface`
source definitions are absent from this checkout and adjacent workspace search;
field storage types, timestamps, and extra fields remain unverified. Before ROS
integration, inspect installed interfaces with `ros2 interface show` rather than
inventing a definition. There is no ROS adapter in Phase 1.

| Topic | Imported message | Observed fields and conventions |
| --- | --- | --- |
| `/vehicle/motor_power` | `boat_iface.msg.MotorPower` | `left_power`, `right_power`, floats normalized to [-1, 1] |
| `/vehicle/sensors/gps` | `boat_iface.msg.GPS` | `latitude`, `longitude` in degrees; `heading` in degrees; `vx`, `vy`, `vz` integers |
| `/vehicle/vehicle_supervisor_state` | `boat_iface.msg.VehicleSupervisorState` | `armed`, `flight_mode`; MANUAL=0, HOLD=4, GUIDED=15 |
| `/mission/goal_waypoint` | `boat_iface.msg.GoalWaypoint` | `x`, `y` geographic coordinates scaled by 1e7 |
| `/vehicle/manual_control` | `boat_iface.msg.ManualControl` | `x`, `y`, `r`, `z`; z uses [0, 1000], x uses [-1000, 1000] in controller |
| `/simulation/mission_items` | `boat_iface.msg.MissionItems` | `items`, containing mission item x/y coordinates |

GPS subscriptions/publishers use `qos_profile_sensor_data`; commands, goals,
and supervisor state use queue depth 10. The GPS driver assigns `vx` from
`velN / 10` and `vy` from `velE / 10`; existing simulation assigns north/east
speed times 100. This establishes north/east ordering and strongly suggests
cm/s units, but formal interface definitions and UBX decoder scaling still
need verification before a new adapter populates velocity fields. `vz` is zero
for this surface vehicle. Neither timestamp fields nor validity flags are
verified in the custom message. Body heading and GPS course must remain distinct
when simulating the selected hardware heading mode.

## Existing simulation and implementation limitations

* `simulation/sensors/sim_gps.py` initializes at the first uploaded mission item,
  then interpolates through remaining waypoints using configured speed. Its
  published velocity comes from speed and bearing, not consecutive GPS fixes.
  It sets velocity to zero when inhibited or finished, without dynamics.
* `simulation/sim_boat.py` duplicates the same `SimGpsNode` and constant-speed
  movement. It adds a motor-power subscription referencing `self.motor_power_cb`,
  but the callback is absent. It cannot form a motor-driven feedback loop.
* `launch/boat_control_launch.py` conditionally starts only `sim_gps` and declares
  an unused `use_sim_boat` argument. It also starts GCS, mission, supervisor,
  and controller. It is not a launch configuration for the new simulator.
* The physical motor node's previously unbound callback is fixed in current
  main to use `self.motor_power_cb`. It opens serial during initialization;
  never instantiate it merely to test an interface.
* `core/state/registry.py` now correctly reads `msg.vx`, `msg.vy`, and `msg.vz`.
  The active controller and mission manager still use their own callbacks and
  supervisor caches rather than this registry.
* `launch/heading_tune_launch.py` is a new hardware launch: it starts GPS,
  heading tuning, and the physical motor driver. It is not a simulation entry
  point and must not be used to run this simulator. The new tuner uses the
  existing heading gains, but is not imported or run by Phase 1.
* `vehicle_controller.py` initializes `self.long` but the GPS callback writes
  `self.lon`, and GUIDED does not guard missing GPS/heading before invoking PID.
  A later integration must ensure valid feedback before providing a goal.
* HOLD/disarmed silence can leave the last nonzero command latched in an
  external receiver. The new core explicitly clears pending commands on mode
  transitions. A later adapter also needs freshness timeouts for dropped state
  or motor messages; these are not core ROS features in Phase 1.

All findings are documented only; simulator development does not edit the
existing implementations.

## Compatibility review after syncing main

The feature branch fast-forwarded from `dc4cc68` to `2460c6a`, incorporating
four upstream commits without conflicts. Upstream changes do not overlap the
staged `aquabot_simulator/` files. Changes to existing files are inherited
from main, not simulator edits.

Reviewed the full upstream diff, including the GPS driver, motor callback,
state registry, GCS telemetry, mission-upload addressing, communication
configuration, heading tuner, launch files, and package entry points.
The standalone core imports only its own modules and the Python standard
library, so none of those implementations is a runtime dependency.

The relevant reference contracts are unchanged: MANUAL=0, HOLD=4, GUIDED=15;
`MotorPower.left_power`/`right_power` and normalized command scaling; GPS
latitude/longitude/heading fields and north/east velocity mapping; waypoint
coordinate scaling; controller mixing; and `CtrlCoeffs` gains. The controller,
supervisor, flight-mode enum, math helpers, coefficient file, and old simulation
modules are unchanged. No Phase 1 dynamics or command-policy changes are needed.
This document corrects previously reported upstream bugs and heading behavior.
The PID turn-sign uncertainty remains unresolved for the later ROS phase.

## PID details and turn-sign uncertainty

`boat_control/data/ctrl_coeffs.py` sets `kp_d=0.1`, `ki_d=kd_d=0`,
`kp_h=0.005`, and `ki_h=kd_h=0`. Proportional units are normalized effort per
metre and per degree respectively. Derivatives use `time.monotonic()` in seconds,
giving normalized effort per (m/s) and per (deg/s). "Integral" terms sum the
previous ten errors, without multiplication by dt, then clamp that sum to
[-50, 50]; they are sample sums rather than conventional time integrals and
depend on call frequency. They omit the current error until the next callback.
Each controller output is clamped to [-1, 1].

Mixing uses `left = distance_control - heading_control` and
`right = distance_control + heading_control`, then normalizes the pair. Positive
heading error therefore requests stronger right thrust. Under conventional port
left/starboard right mounting and clockwise heading, stronger left thrust would
turn clockwise. The physical mounting/polarity is not supplied. Phase 1 retains
the conventional physical torque sign and tests it explicitly. Do not claim
unchanged-controller convergence or silently swap motors to hide this mismatch.
Resolve actual conventions and expose any adapter mapping explicitly in Phase 3.

No PID gains are changed or copied into a new controller. GPS noise, low sampling
rate relative to the 100 Hz controller, monotonic-time derivatives, motor lag,
and unavailable encoder feedback limit future tuning accuracy. Deterministic
controller evaluation will need a clock boundary around the existing PID;
the pure dynamics core itself already uses only simulation time.

## Isolated design and Phase 1 implementation plan

The intended three layers are a pure Python core, a standalone runner, and a
later external ROS adapter. Phase 1 implements the first two, using no repository
imports. Every new file is under `aquabot_simulator/`:

```text
aquabot_simulator/
  .gitignore
  README.md
  requirements.txt
  config/default.yaml
  docs/repository_analysis.md
  aquabot_simulator/
    __init__.py
    vehicle_state.py
    motor_model.py
    dynamics.py
    config.py
    standalone.py
  tests/
    test_motor_model.py
    test_dynamics.py
    test_standalone.py
```

Implementation order: immutable state and units; validated per-motor lag,
dead zone, thrust curve and asymmetry; bounded-step surge/yaw integration with
drag and acceleration limits; fail-closed arming/mode command handling; strict
configuration and standalone CLI; standard-library tests and demonstrations;
final Git and SHA-256 preservation audit. No root manifests, launch files,
message definitions, controllers, or documentation are changed.

Future integration should run an independent Python entry point on a dedicated
ROS domain and simulation topics such as `/aquabot_sim/vehicle/motor_power` and
`/aquabot_sim/vehicle/sensors/gps`, with explicit remaps on the controller,
supervisor, and mission manager. These are proposed names, not existing topics.
Exact execution/remapping instructions will accompany the implemented adapter
after installed message definitions are verified. Never start GPS UART or
physical motor nodes in that domain, and never automatically switch backends.
