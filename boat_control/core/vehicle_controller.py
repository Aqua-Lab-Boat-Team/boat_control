import serial, struct
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached, VehicleSupervisorState, GoalWaypoint, ManualControl, GPS, MotorPower
from boat_iface.srv import ArmDisarm, FlightModeChange
from boat_control.core.mission.Mission import Mission, MissionType
from boat_control.data.supervisor_state_cache import SupervisorStateCache
from boat_control.enums.flight_mode import FlightMode
from boat_control.data.ctrl_coeffs import CtrlCoeffs
from boat_control.core.controllers.thruster_waypoint_pid_ctrl import ThrusterPIDControl
from boat_control.helpers.math_helpers import *
from boat_control.data.comms_config import CommsConfig
from rclpy.qos import qos_profile_sensor_data

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        self.cache = SupervisorStateCache()
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

    def pub_motor_command(self, left_power, right_power):
        left_power = clamp(left_power, self.last_left -1/100, self.last_left + 1/100)
        right_power = clamp(right_power, self.last_right -1/100, self.last_right + 1/100)

        self.last_left = left_power
        self.last_right = right_power

        msg = MotorPower()
        msg.left_power = float(left_power)
        msg.right_power = float(right_power)
        self.self.motor_controller_sim_pub.publish(msg)

    def loop(self):
        if self.cache.arm_state == True:
            match self.cache.flight_mode:
                case FlightMode.HOLD:
                    pass
                case FlightMode.MANUAL:
                    # Execute manual commands
                    left_power = map(self.z, 0, 1000, -1, 1)
                    right_power = map(self.x, -1000, 1000, -1, 1)

                    self.pub_motor_command(left_power, right_power)
                    
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
                    self.pub_motor_command(left_power, right_power)
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
