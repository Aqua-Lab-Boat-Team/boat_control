# AquaBot simulator — Phase 1

A lightweight, deterministic 2D surface-vehicle model driven by left/right
motor commands. Every simulator file lives in this directory. Existing AquaBot
files are reference material and have not been edited.

**Status:** repository analysis and standalone dynamics are implemented and
tested. This follows the specification's instruction to implement Phase 1 only
after initial approval. GPS, CSV logging, ROS integration, waypoint evaluation,
and real-world calibration are subsequent phases, not implemented features.

## Run on Raspberry Pi or a desktop

Requires Python 3.10 or newer. There are no third-party dependencies, ROS
requirements, serial connections, GPU requirements, or Gazebo components.
Run directly from this directory; do not install through the existing root
`setup.py` or alter the existing ROS environment.

```sh
cd aquabot_simulator
python3 -B -m aquabot_simulator.standalone --config config/default.yaml
```

On Windows use `python` in place of `python3`. The default configuration is
disarmed in HOLD, so it remains at rest. Enable **simulated** actuation explicitly:

```sh
python3 -B -m aquabot_simulator.standalone --armed --mode GUIDED --left 0.8 --right 0.8 --duration 20
python3 -B -m aquabot_simulator.standalone --armed --mode MANUAL --left 0.8 --right 0.2 --duration 10
python3 -B -m aquabot_simulator.standalone --forward-speed 2 --left 0 --right 0 --duration 10
```

The first example drives north, the second curves clockwise, and the third
coasts with drag while disarmed. Each prints a final JSON ground-truth snapshot.
Use `--heading 90` to start pointing east, `--east` / `--north` to set position,
`--yaw-rate` to set initial yaw, and `--dt` to select the step. Use `--realtime`
to pace execution by wall time; by default simulation runs as fast as possible.
`--no-armed` explicitly overrides an armed configuration. See `--help` for all
options. Commands and modes never access hardware.

An optional isolated environment needs no package installation:

```sh
python3 -m venv .venv
. .venv/bin/activate
python3 -B -m unittest discover -s tests -v
```

Creating the environment is optional; tests also run directly with the last
command. On Windows activate with `.venv\Scripts\Activate.ps1`.

## Configuration and API

`config/default.yaml` uses JSON syntax, which is a YAML 1.2 subset. The loader
accepts that syntax only, allowing a standard-library implementation without
PyYAML. Preserve braces, quoted keys, and commas when editing. General YAML
indentation syntax and comments are not supported. Unknown/duplicate keys,
nonfinite numbers, invalid model parameters, and nonboolean arming are rejected.
Omitted sections use safe defaults.

```python
from aquabot_simulator import FlightMode, Simulator, VehicleState

sim = Simulator(initial=VehicleState(heading_deg=90))
sim.set_operating_state(True, FlightMode.GUIDED)
sim.set_commands(0.8, 0.8)
state = sim.step(0.02)
sim.set_operating_state(False, FlightMode.HOLD)
coasting = sim.step(0.02)
```

The core accepts no target waypoint. Position emerges entirely from thrust,
drag, and integration. `state` is an immutable snapshot of the last completed
step. `set_commands` updates the next step's input. The standalone `simulate`
function yields snapshots incrementally and stores no trajectory, keeping
memory use independent of duration. JSON console output is ground truth for
inspection, not GPS or controller feedback. CSV logging is reserved for Phase 2.

## Model assumptions and units

All default coefficients are illustrative, uncalibrated values. They are not
measured thrust, encoder RPM, or evidence of valid PID gains for a real boat.

* Coordinates: east/north in metres; heading in degrees clockwise from north,
  normalized to `[0, 360)`. Signed heading errors use `[-180, 180)`.
* Motion: one body-forward velocity in m/s and clockwise yaw in degrees/s.
  No sway, current, wind, inertia coupling, hull geometry, or environmental
  disturbance is modeled. Reverse commands produce negative forward velocity.
* Inputs: normalized commands saturate at `[-1, 1]`. The hardware driver's
  additional 0.75 scaling is not silently applied; tune the model's thrust limit
  during calibration if that mapping is relevant.
* Motor dead zone: normalized magnitude at or below `dead_zone` targets zero;
  the remaining range is linearly rescaled to `[0, 1]`.
* Motor response: a dimensionless effective state follows the rescaled command
  with first-order lag, `alpha = 1 - exp(-dt / response_time_s)`. A zero time
  constant selects instantaneous response explicitly. This state is not RPM.
* Thrust: `max_thrust_n * efficiency * sign(state) * abs(state)**exponent`.
  Left/right coefficients can differ. Each motor's effective thrust limit is
  `max_thrust_n * efficiency` N. The default exponent is 2.
* Forward force: `left_thrust + right_thrust` N, divided by `mass_kg`.
  Water resistance is `linear_drag*v + quadratic_drag*v*abs(v)` N.
* Clockwise torque: `(left_thrust - right_thrust) * thruster_separation_m / 2`
  N m. It is divided by `yaw_inertia_kg_m2`. Yaw damping acts on radians/s.
  A physically port-mounted forward thruster increases clockwise heading.
* Drag coefficients have units N/(m/s), N/(m/s)^2, N m/(rad/s), and
  N m/(rad/s)^2 respectively. Mass is kg, yaw inertia kg m^2, separation metres.
* Forward and yaw acceleration limits bound each velocity change per substep,
  including deceleration. Surge and yaw drag use an implicit scalar solve to
  avoid drag alone reversing velocity. Position and heading use trapezoidal
  velocities and midpoint heading.
* Core calls accept `0 < dt <= 60` seconds and subdivide uniformly. Internal
  `max_substep_s` must be in `[0.001, 0.1]`, default 0.02 (50 Hz). The runner's
  requested step is in `[0.001, 60]` seconds. It includes a partial final step.
  Smaller steps improve accuracy; this is a simplified numerical model.

## Arming and mode policy

Default: disarmed, HOLD, zero effective actuation. Only armed MANUAL (0) or
GUIDED (15) accepts commands, matching the modes that publish motor commands
in the existing controller. HOLD (4), disarming, and unsupported integer modes
inhibit commands. Any operating-state transition clears the pending commands,
so rearming requires a fresh command. Repeated unchanged supervisor state
does not clear a valid command. Commands sent while inhibited are discarded.

Inhibition requests zero motor output; residual motor state decays with motor
lag and vehicle momentum decays with water drag. It does not freeze pose or
instantaneously zero velocity. Hardware drivers are never imported, and the
simulator never automatically switches to physical hardware.

## Repository findings and next phases

See [repository analysis](docs/repository_analysis.md) for actual topics, data
flow, existing simulation limitations, PID behavior, and integration uncertainties.
The package is independently runnable through `python -m`; no empty ROS/GPS
placeholder modules or root-package changes are needed for Phase 1.

Compatibility was reviewed against main commit `2460c6a`. The latest motor
callback and GPS registry fixes, GPS heading changes, GCS/mission updates, and
new heading tuner require no Phase 1 dynamics changes: this core imports none
of those nodes. Flight modes and the referenced motor/GPS conventions remain
unchanged. The repository analysis reflects the updated source. The new
`heading_tune_launch.py` starts physical hardware and is not a simulator launch.

1. Phase 2: geographic conversion, seeded GPS noise, measurement timing and
   latency, GPS-derived velocity with invalid-time guards, optional CSV logging.
2. Phase 3: an external ROS adapter after verifying the installed `boat_iface`
   definitions. Use isolated simulation topics, explicit controller remaps,
   supervisor/command freshness timeouts, and no physical nodes. Preserve sensor
   QoS. Resolve motor turn sign through a documented adapter option or measured
   mounting convention before closing the loop.
3. Phase 4: evaluate the unchanged PID using measured simulated sensor feedback;
   record acceptance, cross-track error, overshoot, settling, oscillation,
   saturation, and command effort. Existing mission completion and stopping
   limitations must be accounted for externally.
4. Phase 5: calibrate against real timestamped commands and GPS logs, then
   compare candidate gains. No real logs were supplied, so calibration and
   physical PID validation cannot yet be claimed. Do not automate gain search
   before validating the dynamics and feedback loop.

## Validation

The Phase 1 suite covers forward/reverse motion, heading frames, differential
turning, motor lag, asymmetry, thrust/command limits, dead zones, drag, coasting,
acceleration limits, arming/HOLD/unsupported modes, stale-command clearing,
immutable snapshots, numerical substeps, invalid inputs, partial final steps,
repeatability, CLI execution, and blocked hardware/ROS imports in a fresh
process. GPS and waypoint tests belong to their subsequent implementation phases.
