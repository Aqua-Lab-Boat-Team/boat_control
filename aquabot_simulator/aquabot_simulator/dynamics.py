"""Surge and yaw model with implicit water damping and bounded substeps."""

from dataclasses import dataclass, fields, replace
import math

from .motor_model import MotorConfig, MotorModel
from .vehicle_state import FlightMode, VehicleState, finite


@dataclass(frozen=True)
class DynamicsConfig:
    mass_kg: float = 25.0
    yaw_inertia_kg_m2: float = 8.0
    thruster_separation_m: float = 0.6
    linear_drag_n_per_mps: float = 4.0
    quadratic_drag_n_per_mps2: float = 8.0
    yaw_drag_nm_per_rad_s: float = 4.0
    yaw_quadratic_drag_nm_per_rad_s2: float = 2.0
    max_acceleration_mps2: float = 2.0
    max_yaw_acceleration_deg_s2: float = 90.0
    max_substep_s: float = 0.02

    def __post_init__(self):
        positive = {"mass_kg", "yaw_inertia_kg_m2", "max_acceleration_mps2",
                    "max_yaw_acceleration_deg_s2", "max_substep_s"}
        for field in fields(self):
            value = finite(field.name, getattr(self, field.name))
            if value < 0 or (field.name in positive and value == 0):
                raise ValueError(f"invalid {field.name}: {value}")
        if not 0.001 <= self.max_substep_s <= 0.1:
            raise ValueError("max_substep_s must be in [0.001, 0.1] s")


def _damped_velocity(old, force, inertia, linear, quadratic, dt, limit):
    # Solve implicit drag: a*v*abs(v) + b*v = old + dt*force/inertia.
    # The rationalized quadratic root avoids cancellation near zero.
    free = old + dt * force / inertia
    a = dt * quadratic / inertia
    b = 1.0 + dt * linear / inertia
    magnitude = 2.0 * abs(free) / (b + math.sqrt(b*b + 4*a*abs(free)))
    damped = math.copysign(magnitude, free)
    change = max(-limit * dt, min(limit * dt, damped - old))
    return old + change


class Simulator:
    """A reusable, hardware-free simulation, initially disarmed in HOLD.

    Calls to step return immutable snapshots. Only armed MANUAL/GUIDED accept
    commands. Safety transitions clear commands; momentum and motor lag decay.
    """

    def __init__(self, config: DynamicsConfig | None = None,
                 left_motor: MotorConfig | None = None,
                 right_motor: MotorConfig | None = None,
                 initial: VehicleState | None = None):
        self.config = config or DynamicsConfig()
        initial = initial or VehicleState()
        # Initial conditions specify pose and momentum, never latent actuation.
        if any((initial.left_command, initial.right_command,
                initial.left_motor_state, initial.right_motor_state,
                initial.left_thrust_n, initial.right_thrust_n)):
            raise ValueError("initial actuation must be zero")
        self._state = initial
        self._left_motor = MotorModel(left_motor)
        self._right_motor = MotorModel(right_motor)
        self._left = self._right = 0.0
        self._armed = False
        self._mode = int(FlightMode.HOLD)

    @property
    def state(self) -> VehicleState:
        return self._state

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def mode(self) -> int:
        return self._mode

    @property
    def accepts_commands(self) -> bool:
        return self.armed and self.mode in (FlightMode.MANUAL, FlightMode.GUIDED)

    def set_operating_state(self, armed: bool, mode: int) -> None:
        if not isinstance(armed, bool):
            raise ValueError("armed must be a boolean")
        if isinstance(mode, bool) or not isinstance(mode, int):
            raise ValueError("mode must be an integer flight mode")
        if armed != self.armed or mode != self.mode:
            self._left = self._right = 0.0
        self._armed, self._mode = armed, int(mode)

    def set_commands(self, left: float, right: float) -> None:
        left, right = finite("left", left), finite("right", right)
        if self.accepts_commands:
            self._left = max(-1.0, min(1.0, left))
            self._right = max(-1.0, min(1.0, right))
        else:
            self._left = self._right = 0.0

    def step(self, dt_s: float) -> VehicleState:
        dt_s = finite("dt_s", dt_s)
        if dt_s <= 0 or dt_s > 60:
            raise ValueError("dt_s must be in (0, 60] seconds")
        count = math.ceil(dt_s / self.config.max_substep_s)
        dt = dt_s / count
        end_time = self.state.time_s + dt_s
        for _ in range(count):
            self._advance(dt)
        # Avoid substep accumulation affecting the requested sample timestamp.
        self._state = replace(self.state, time_s=end_time)
        return self.state

    def _advance(self, dt):
        cfg, old = self.config, self.state
        left = self._left if self.accepts_commands else 0.0
        right = self._right if self.accepts_commands else 0.0
        thrust_left = self._left_motor.step(left, dt)
        thrust_right = self._right_motor.step(right, dt)
        velocity = _damped_velocity(
            old.forward_mps, thrust_left + thrust_right, cfg.mass_kg,
            cfg.linear_drag_n_per_mps, cfg.quadratic_drag_n_per_mps2,
            dt, cfg.max_acceleration_mps2)
        # Port (left) thrust exceeds starboard -> clockwise yaw from north.
        torque = (thrust_left - thrust_right) * cfg.thruster_separation_m / 2
        yaw = _damped_velocity(
            math.radians(old.yaw_rate_deg_s), torque, cfg.yaw_inertia_kg_m2,
            cfg.yaw_drag_nm_per_rad_s, cfg.yaw_quadratic_drag_nm_per_rad_s2,
            dt, math.radians(cfg.max_yaw_acceleration_deg_s2))
        heading_delta = (old.yaw_rate_deg_s + math.degrees(yaw)) * dt / 2
        midpoint_heading = math.radians(old.heading_deg + heading_delta / 2)
        distance = (old.forward_mps + velocity) * dt / 2
        self._state = VehicleState(
            time_s=old.time_s + dt,
            east_m=old.east_m + distance * math.sin(midpoint_heading),
            north_m=old.north_m + distance * math.cos(midpoint_heading),
            heading_deg=(old.heading_deg + heading_delta) % 360,
            forward_mps=velocity, yaw_rate_deg_s=math.degrees(yaw),
            left_command=left, right_command=right,
            left_motor_state=self._left_motor.state,
            right_motor_state=self._right_motor.state,
            left_thrust_n=thrust_left, right_thrust_n=thrust_right)
