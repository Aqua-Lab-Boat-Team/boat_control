"""Local ground-truth state, never implicitly exposed as sensor feedback."""

from dataclasses import dataclass, fields
from enum import IntEnum
import math


def finite(name: str, value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f"{name} must be a finite number")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def wrap_angle_deg(angle: float) -> float:
    """Signed shortest angle, in [-180, 180)."""
    return (finite("angle", angle) + 180.0) % 360.0 - 180.0


class FlightMode(IntEnum):
    """Verified values from boat_control/enums/flight_mode.py."""

    MANUAL = 0
    HOLD = 4
    GUIDED = 15


@dataclass(frozen=True)
class VehicleState:
    time_s: float = 0.0
    east_m: float = 0.0
    north_m: float = 0.0
    heading_deg: float = 0.0
    forward_mps: float = 0.0
    yaw_rate_deg_s: float = 0.0
    left_command: float = 0.0
    right_command: float = 0.0
    left_motor_state: float = 0.0
    right_motor_state: float = 0.0
    left_thrust_n: float = 0.0
    right_thrust_n: float = 0.0

    def __post_init__(self):
        for field in fields(self):
            finite(field.name, getattr(self, field.name))
        if self.time_s < 0:
            raise ValueError("time_s must be non-negative")
        for name in ("left_command", "right_command", "left_motor_state",
                     "right_motor_state"):
            if abs(getattr(self, name)) > 1:
                raise ValueError(f"{name} must be in [-1, 1]")
        object.__setattr__(self, "heading_deg", self.heading_deg % 360.0)

    @property
    def east_velocity_mps(self) -> float:
        return self.forward_mps * math.sin(math.radians(self.heading_deg))

    @property
    def north_velocity_mps(self) -> float:
        return self.forward_mps * math.cos(math.radians(self.heading_deg))
