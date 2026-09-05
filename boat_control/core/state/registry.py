from dataclasses import dataclass
from typing import Callable, Generic, TypeVar

from rclpy.qos import QoSProfile, ReliabilityPolicy

from boat_control.core.state.definitions import *
from boat_iface.msg import VehicleSupervisorState, GPS, ManualControl
from boat_control.enums.flight_mode import FlightMode

StateT = TypeVar("StateT")
MsgT = TypeVar("MsgT")


@dataclass(frozen=True)
class StateSpec(Generic[StateT, MsgT]):
    state_type: type[StateT]
    msg_type: type[MsgT]
    topic: str
    qos: QoSProfile
    update: Callable[[StateT, MsgT], None]

state_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

def update_supervisor_state(state: SupervisorState, msg: VehicleSupervisorState):
    state.armed = msg.armed
    state.flight_mode = FlightMode(msg.flight_mode)


def update_gps(state: GPSState, msg: GPS):
    state.latitude = msg.latitude
    state.longitude = msg.longitude
    state.heading = msg.heading
    state.vx = vx
    state.vy = vy
    state.vz = vz


STATE_REGISTRY = {
    SupervisorState: StateSpec(
        state_type=SupervisorState,
        msg_type=VehicleSupervisorState,
        topic='/vehicle/vehicle_supervisor_state',
        qos=state_qos,
        update=update_supervisor_state,
    ),

    GPSState: StateSpec(
        state_type=GPSState,
        msg_type=GPS,
        topic="/vehicle/sensors/gps",
        qos=sensor_qos,
        update=update_gps,
    ),
}
