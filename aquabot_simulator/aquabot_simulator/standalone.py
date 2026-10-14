"""Run deterministic motor-driven dynamics without ROS or serial imports."""

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import time

from .config import RunConfig, load_config
from .dynamics import Simulator
from .vehicle_state import FlightMode


def simulate(config: RunConfig):
    """Yield one snapshot at a time; memory use is independent of duration."""
    sim = Simulator(config.dynamics, config.left_motor, config.right_motor,
                    config.initial)
    sim.set_operating_state(config.armed, config.mode)
    sim.set_commands(config.left, config.right)
    yield sim.state
    elapsed = 0.0
    while elapsed < config.duration_s:
        remaining = config.duration_s - elapsed
        # Include a partial final step without a floating-point extra step.
        dt = min(config.dt_s, remaining)
        if remaining <= config.dt_s * 1e-10:
            break
        yield sim.step(dt)
        elapsed = sim.state.time_s - config.initial.time_s


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).resolve().parents[1]
                        / "config" / "default.yaml")
    parser.add_argument("--duration", type=float, help="simulation seconds")
    parser.add_argument("--dt", type=float, help="integration step in seconds")
    parser.add_argument("--left", type=float, help="normalized left command")
    parser.add_argument("--right", type=float, help="normalized right command")
    parser.add_argument("--armed", action=argparse.BooleanOptionalAction,
                        default=None, help="enable simulated actuation")
    parser.add_argument("--mode", choices=list(FlightMode.__members__))
    parser.add_argument("--east", type=float, help="initial east position, m")
    parser.add_argument("--north", type=float, help="initial north position, m")
    parser.add_argument("--heading", type=float, help="clockwise from north, deg")
    parser.add_argument("--forward-speed", type=float, help="initial surge, m/s")
    parser.add_argument("--yaw-rate", type=float, help="initial clockwise deg/s")
    parser.add_argument("--realtime", action="store_true",
                        help="pace execution by wall time (default runs fast)")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        overrides = {name: value for name, value in (
            ("duration_s", args.duration), ("dt_s", args.dt),
            ("left", args.left), ("right", args.right), ("armed", args.armed))
                     if value is not None}
        if args.mode is not None:
            overrides["mode"] = FlightMode[args.mode]
        initial = {name: value for name, value in (
            ("east_m", args.east), ("north_m", args.north),
            ("heading_deg", args.heading), ("forward_mps", args.forward_speed),
            ("yaw_rate_deg_s", args.yaw_rate)) if value is not None}
        config = replace(config, initial=replace(config.initial, **initial),
                         **overrides)
        start = time.monotonic()
        for final in simulate(config):
            if args.realtime:
                delay = final.time_s - config.initial.time_s - (time.monotonic() - start)
                if delay > 0:
                    time.sleep(delay)
        print(json.dumps({"simulation_only": True, "armed": config.armed,
                          "mode": config.mode.name, "state": asdict(final)},
                         indent=2, allow_nan=False))
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
