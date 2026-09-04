import serial, struct
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached, VehicleSupervisorState, GoalWaypoint, ManualControl, GPS, MotorPower
from boat_iface.srv import ArmDisarm, FlightModeChange
from boat_control.Mission import Mission, MissionType
from boat_control.data.supervisor_state_cache import SupervisorStateCache
from boat_control.enums.flight_mode import FlightMode
from boat_control.data.ctrl_coeffs import CtrlCoeffs
from boat_control.controllers.thruster_waypoint_pid_ctrl import ThrusterPIDControl
from boat_control.helpers.math_helpers import *
from boat_control.data.comms_config import CommsConfig
from rclpy.qos import qos_profile_sensor_data

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        self.cache = SupervisorStateCache()

        #### PARAMETERS ####
        self.declare_parameter('use_sim_boat', False)
        self.use_sim_boat = self.get_parameter(
            'use_sim_boat'
        ).get_parameter_value().bool_value
        ####################

        if not self.use_sim_boat:
            self.declare_parameter("port", CommsConfig.ARDUINO_UART_PORT)
            self.declare_parameter("baudrate", CommsConfig.ARDUINO_UART_BAUD)
            port = str(self.get_parameter("port").value)
            baudrate = int(self.get_parameter("baudrate").value)
        else:
            self.motor_controller_sim_pub = self.create_publisher(MotorPower, '/vehicle/motor_power', 10)

        ### Local state ###
        self.lat: int | None = None
        self.long: int | None = None
        self.hdg: float | None = None

        self.goal_lat: int | None = None
        self.goal_lon: int | None = None

        self.x = 0
        self.y = 0
        self.r = 0
        self.z = 0

        self.last_left = 0
        self.last_right = 0
        ###################

        ### Member objects ###
        self.controller = ThrusterPIDControl(CtrlCoeffs)
        ######################
        
        if not self.use_sim_boat:
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
                f"Arduino UART {port} at {baudrate} baud"
            )
        else:
            self.get_logger().info("Connected to virtual boat")
        
        ### SUBSCRIPTIONS ###
        self.vehicle_supervisor_state_sub = self.create_subscription(
            VehicleSupervisorState,
            '/vehicle/vehicle_supervisor_state',
            self.vehicle_supervisor_state_sub_cb,
            10,
        )

        self.gps_sub = self.create_subscription(GPS, '/vehicle/sensors/gps', self.gps_sub_cb, qos_profile_sensor_data)
        self.goal_sub = self.create_subscription(GoalWaypoint, '/mission/goal_waypoint', self.waypoint_sub_cb, 10)
        self.man_ctrl_sub = self.create_subscription(ManualControl, '/vehicle/manual_control', self.manual_control_sub_cb, 10)
        ######################
        # left_power, right_power = self.controller.calc_control(
        #             self.lat,
        #             self.lon,
        #             self.goal_lat,
        #             self.goal_lon,
        #             self.hdg
        #         )
        self.timer = self.create_timer(0.01, self.loop)

    def manual_control_sub_cb(self, msg) -> None:
        self.x = msg.x
        self.y = msg.y
        self.r = msg.r 
        self.z = msg.z 

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
        left_power = clamp(left_power, self.last_left -1/100, self.last_left + 1/100)
        right_power = clamp(right_power, self.last_right -1/100, self.last_right + 1/100)

        ### Change to map instead of clamp to avoid saturating the controller
        # left_power = clamp(left_power, -0.75, 0.75)
        # right_power = clamp(right_power, -0.75, 0.75)

        left_power = map(left_power, -1, 1, -0.75, 0.75)
        right_power = map(right_power, -1, 1, -0.75, 0.75)

        self.last_left = left_power
        self.last_right = right_power
        
        left_power = int_map(left_power, -1, 1, -128, 127)
        right_power = int_map(right_power, -1, 1, -128, 127)
        
        command = f"{left_power} {right_power}\n"
        self.get_logger().info(command)

        if not self.use_sim_boat:
            self.serial_port.write( command.encode("ascii") )
        else:
            msg = MotorPower()
            msg.left_power = left_power
            msg.right_power = right_power
            self.self.motor_controller_sim_pub.publish(msg)

    def loop(self):
        self.get_logger().info(f"MODE: {self.cache.flight_mode}")
        if self.cache.arm_state == True:
            match self.cache.flight_mode:
                case FlightMode.HOLD:
                    pass
                case FlightMode.MANUAL:
                    # Execute manual commands
                    left_power = map(self.z, 0, 1000, -1, 1)
                    right_power = map(self.x, -1000, 1000, -1, 1)

                    self.send_motor_command(left_power, right_power)
                    
                case FlightMode.GUIDED:
                    
                    # Path following controller
                    # Outputs -1 to 1
                    if self.goal_lat != None and self.goal_lon !=None:
                        left_power, right_power = self.controller.calc_control(
                            self.lat,
                            self.lon,
                            self.goal_lat,
                            self.goal_lon,
                            self.hdg
                        )
                    else:
                        left_power = 0
                        right_power = 0
                    self.get_logger().info(f"L: {left_power}, R: {right_power}")
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
