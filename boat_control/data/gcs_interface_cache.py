from dataclasses import dataclass
from boat_control.enums.flight_mode import FlightMode

@dataclass
class GCSInterfaceCache:
    arm_state:bool = False
    flight_mode:FlightMode = FlightMode.HOLD
