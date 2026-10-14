"""Strict configuration using JSON syntax (a subset of YAML 1.2)."""

from dataclasses import dataclass, field
import json
from pathlib import Path

from .dynamics import DynamicsConfig
from .motor_model import MotorConfig
from .vehicle_state import FlightMode, VehicleState, finite


@dataclass(frozen=True)
class RunConfig:
    duration_s: float = 20.0
    dt_s: float = 0.02
    initial: VehicleState = field(default_factory=VehicleState)
    dynamics: DynamicsConfig = field(default_factory=DynamicsConfig)
    left_motor: MotorConfig = field(default_factory=MotorConfig)
    right_motor: MotorConfig = field(default_factory=MotorConfig)
    armed: bool = False
    mode: FlightMode = FlightMode.HOLD
    left: float = 0.5
    right: float = 0.5

    def __post_init__(self):
        duration = finite("duration_s", self.duration_s)
        dt = finite("dt_s", self.dt_s)
        if duration < 0:
            raise ValueError("duration_s must be non-negative")
        if not 0.001 <= dt <= 60:
            raise ValueError("dt_s must be in [0.001, 60] seconds")
        if not isinstance(self.armed, bool):
            raise ValueError("armed must be a boolean")
        if not isinstance(self.mode, FlightMode):
            raise ValueError("mode must be HOLD, MANUAL, or GUIDED")
        finite("left", self.left)
        finite("right", self.right)


def _object(value, keys, name):
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    unknown = set(value) - set(keys)
    if unknown:
        raise ValueError(f"unknown {name} keys: {', '.join(sorted(unknown))}")
    return value


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate configuration key: {key}")
        result[key] = value
    return result


def load_config(path: str | Path) -> RunConfig:
    """Load the documented JSON-style YAML configuration without PyYAML."""
    with Path(path).open(encoding="utf-8") as stream:
        data = json.load(stream, object_pairs_hook=_unique_pairs)
    _object(data, ("simulation", "initial", "operation", "commands",
                   "dynamics", "left_motor", "right_motor"), "configuration")
    simulation = _object(data.get("simulation", {}),
                         ("duration_s", "dt_s"), "simulation")
    initial = _object(data.get("initial", {}),
                      ("east_m", "north_m", "heading_deg", "forward_mps",
                       "yaw_rate_deg_s"), "initial")
    operation = _object(data.get("operation", {}), ("armed", "mode"), "operation")
    commands = _object(data.get("commands", {}), ("left", "right"), "commands")
    mode_name = operation.get("mode", "HOLD")
    if not isinstance(mode_name, str) or mode_name not in FlightMode.__members__:
        raise ValueError("operation.mode must be HOLD, MANUAL, or GUIDED")
    models = {}
    for name, cls in (("dynamics", DynamicsConfig), ("left_motor", MotorConfig),
                      ("right_motor", MotorConfig)):
        section = _object(data.get(name, {}), cls.__dataclass_fields__, name)
        models[name] = cls(**section)
    return RunConfig(**simulation, initial=VehicleState(**initial), **models,
                     armed=operation.get("armed", False),
                     mode=FlightMode[mode_name], **commands)
