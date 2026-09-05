from dataclasses import dataclass
from boat_control.enums.flight_mode import FlightMode

@dataclass
class SupervisorState:
    armed:bool = False
    flight_mode:FlightMode = FlightMode.HOLD

@dataclass
class GPSState:
    latitude:float = 0.0
    longitude:float = 0.0
    heading:float = 0.0
    vx:int = 0
    vy:int = 0
    vz:int = 0

