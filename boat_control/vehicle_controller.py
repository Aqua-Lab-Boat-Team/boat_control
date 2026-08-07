import serial
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached, VehicleSupervisorState
from boat_iface.srv import ArmDisarm, FlightModeChange
from boat_control.Mission import Mission, MissionType
from boat_control.data.supervisor_state_cache import SupervisorStateCache
from boat_control.enums.flight_mode import FlightMode

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        self.cache = SupervisorStateCache()

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
        except serial.SerialException as error:
            self.get_logger().fatal(
                f"Could not open vehicle controller UART {port}: {error}"
            )
            raise

        self.get_logger().info(
            f"Vehicle controller UART connected on {port} at {baudrate} baud"
        )

        self.vehicle_supervisor_state_sub = self.create_subscription(
            VehicleSupervisorState,
            '/vehicle/vehicle_supervisor_state',
            self.vehicle_supervisor_state_sub_cb,
            10,
        )
        self.timer = self.create_timer(0.01, self.loop)

    def vehicle_supervisor_state_sub_cb(self, msg):
        self.cache.arm_state = msg.armed
        self.cache.flight_mode = FlightMode(msg.flight_mode)

    def loop(self):
        if not self.cache.arm_state:
            return

        match self.cache.flight_mode:
            case FlightMode.HOLD:
                pass
            case FlightMode.MANUAL:
                pass
            case FlightMode.GUIDED:
                pass
            case _:
                pass

    def destroy_node(self) -> None:
        if hasattr(self, "serial_port") and self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    vehicle_controller = VehicleController()

    rclpy.spin(vehicle_controller)

    vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
