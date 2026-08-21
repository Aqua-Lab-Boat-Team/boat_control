import serial, struct
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached, VehicleSupervisorState, GoalWaypoint
from boat_iface.srv import ArmDisarm, FlightModeChange
from boat_control.Mission import Mission, MissionType
from boat_control.data.supervisor_state_cache import SupervisorStateCache
from boat_control.enums.flight_mode import FlightMode
from boat_control.data.ctrl_coeffs import CtrlCoeffs
from boat_control.controllers.thruster_waypoint_pid_ctrl import ThrusterPIDControl

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        self.cache = SupervisorStateCache()

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)

        ### Local state ###
        self.lat: int | None = None
        self.long: int | None = None
        self.hdg: float | None = None

        self.goal_lat: int | None = None
        self.goal_lon: int | None = None
        ###################

        ### Member objects ###
        self.controller = ThrusterPIDControl(CtrlCoeffs)
        ######################
        
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
        
        ### SUBSCRIPTIONS ###
        self.vehicle_supervisor_state_sub = self.create_subscription(
            VehicleSupervisorState,
            '/vehicle/vehicle_supervisor_state',
            self.vehicle_supervisor_state_sub_cb,
            10,
        )

        self.gps_sub = self.create_subscription(GPS, '/vehicle/sensors/gps', self.gps_sub_cb, qos_profile_sensor_data)
        self.goal_sub = self.create_subscription(GoalWaypoint, '/mission/goal_waypoint', self.waypoint_sub_cb, 10)
        ######################
        
        self.timer = self.create_timer(0.01, self.loop)

    def vehicle_supervisor_state_sub_cb(self, msg) -> None:
        self.cache.arm_state = msg.armed
        self.cache.flight_mode = FlightMode(msg.flight_mode)

    def gps_sub_cb(self, msg) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.hdg = msg.heading

    def waypoint_sub_cb(self, msg) -> None:
        self.goal_lat = msg.x / 1e7
        self.goal_lon = msg.y / 1e7

    def send_motor_command(self, left_power, right_power):
        left_power = int_map(left_power, -1, 1, -128, 127)
        right_power = int_map(right_power, -1, 1, -128, 127)

        self.serial_port.write(b'L' + struct.pack('b', left_power))
        self.serial_port.write(b'R' + struct.pack('b', righ_power))

    def loop(self):
        if not self.cache.arm_state:
            return

        match self.cache.flight_mode:
            case FlightMode.HOLD:
                pass
            case FlightMode.MANUAL:
                # Execute manual commands
                pass
            case FlightMode.GUIDED:
                
                # Path following controller

                # Outputs -1 to 1
                left_power, right_power = controller.calc_control(
                    self.lat,
                    self.lon,
                    self.goal_lat,
                    self.goal_lon,
                    self.hdg
                )
                
                # Sends to ESC. Scales to appropriate range
                self.send_motor_command(left_power, right_power)
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
