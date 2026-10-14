"""First-order effective motor response; neither RPM nor measured thrust."""

from dataclasses import dataclass, fields
import math

from .vehicle_state import finite


@dataclass(frozen=True)
class MotorConfig:
    response_time_s: float = 0.4
    max_thrust_n: float = 20.0
    efficiency: float = 1.0
    dead_zone: float = 0.05
    exponent: float = 2.0

    def __post_init__(self):
        for field in fields(self):
            value = finite(field.name, getattr(self, field.name))
            if value < 0:
                raise ValueError(f"{field.name} must be non-negative")
        if self.dead_zone >= 1:
            raise ValueError("dead_zone must be less than 1")
        if self.exponent < 1:
            raise ValueError("exponent must be at least 1")
        if not math.isfinite(self.max_thrust_n * self.efficiency):
            raise ValueError("effective thrust limit must be finite")


class MotorModel:
    def __init__(self, config: MotorConfig | None = None,
                 initial_state: float = 0.0):
        self.config = config or MotorConfig()
        self.state = finite("initial_state", initial_state)
        if abs(self.state) > 1:
            raise ValueError("initial_state must be in [-1, 1]")

    @property
    def thrust_n(self) -> float:
        cfg = self.config
        return (cfg.max_thrust_n * cfg.efficiency
                * math.copysign(abs(self.state) ** cfg.exponent, self.state))

    def step(self, command: float, dt_s: float) -> float:
        command = finite("command", command)
        dt_s = finite("dt_s", dt_s)
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        command = max(-1.0, min(1.0, command))
        cfg = self.config
        magnitude = max(0.0, (abs(command) - cfg.dead_zone)
                        / (1.0 - cfg.dead_zone))
        target = math.copysign(magnitude, command)
        alpha = (1.0 if cfg.response_time_s == 0
                 else -math.expm1(-dt_s / cfg.response_time_s))
        self.state += alpha * (target - self.state)
        return self.thrust_n
