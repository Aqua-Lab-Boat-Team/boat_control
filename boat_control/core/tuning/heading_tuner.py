import serial, struct
import rclpy
import time
import math
import numpy as np
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import GPS, MotorPower, HDGTest
from boat_control.data.ctrl_coeffs import CtrlCoeffs
from boat_control.helpers.math_helpers import *
from boat_control.data.comms_config import CommsConfig
from rclpy.qos import qos_profile_sensor_data
from boat_control.data.ctrl_coeffs import CtrlCoeffs
from boat_control.helpers.math_helpers import *

# ros2 topic pub --once /testing/goal_heading boat_iface/msg/HDGTest '{hdg: 90}'

class HeadingTuner(Node):
    def __init__(self):
        super().__init__('heading_tuner')
        
        ### Local state ###
        self.lat: int | None = None
        self.long: int | None = None
        self.hdg: float | None = None

        self.goal_hdg = float | None
        self.hdg_err_buff = np.zeros(10) # Index 0 is most recent
        self.coeffs = CtrlCoeffs
        self.last_ctrl_time = None
        ###################

        self.gps_sub = self.create_subscription(GPS, '/vehicle/sensors/gps', self.gps_sub_cb, qos_profile_sensor_data)
        self.goal_heading_sub = self.create_subscription(HDGTest ,'/testing/goal_heading', self.goal_heading_sub, 10)

        self.timer = self.create_timer(0.01, self.loop)

    def gps_sub_cb(self, msg) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.hdg = msg.heading

    def goal_heading_sub(self, msg) -> None:
        self.goal_hdg = msg.hdg
        self.get_logger().info(f"New Goal Heading:{self.goal_hdg} ")

    def pub_motor_command(self, left_power, right_power):
        left_power = clamp(left_power, self.last_left -1/100, self.last_left + 1/100)
        right_power = clamp(right_power, self.last_right -1/100, self.last_right + 1/100)

        self.last_left = left_power
        self.last_right = right_power

        msg = MotorPower()
        msg.left_power = float(left_power)
        msg.right_power = float(right_power)
        self.motor_controller_sim_pub.publish(msg)

    def loop(self):
        if self.goal_hdg is not None and self.hdg is not None:
            now = time.monotonic()
            hdg_err = wrap_angle_deg(self.goal_hdg - self.hdg)
            self.get_logger().info(f"-------")
            self.get_logger().info(f"{time.now()}")
            self.get_logger().info(f"Heading: {self.hdg}")
            self.get_logger().info(f"Goal Heading: {self.goal_hdg}")
            self.get_logger().info(f"Error: {hdg_err}")

            int_hdg_err = clamp(np.sum(self.hdg_err_buff), -50, 50)
            d_hdg_err_dt = 0.0
            if self.last_ctrl_time is not None:
                dt = now - self.last_ctrl_time
                hdg_err_delta = wrap_angle_deg(hdg_err - self.hdg_err_buff[0])
                d_hdg_err_dt = hdg_err_delta / dt

            hdg_ctrl = (self.coeffs.kp_h * hdg_err + 
                self.coeffs.ki_h * int_hdg_err + 
                self.coeffs.kd_h * d_hdg_err_dt)

            hdg_ctrl = clamp(hdg_ctrl, -1, 1) # Clamping

            self.hdg_err_buff = np.roll(self.hdg_err_buff, 1)
            self.hdg_err_buff[0] = hdg_err

            self.last_ctrl_time = now
            
            # Mixing
            left_power = -hdg_ctrl
            right_power = hdg_ctrl

            norm_factor = max(1, max(abs(left_power), abs(right_power)))
            left_power = left_power / norm_factor
            right_power = right_power / norm_factor

            self.pub_motor_command(left_power, right_power)


    def destroy_node(self) -> None:
        if hasattr(self, "serial_port") and self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    heading_tuner = HeadingTuner()

    rclpy.spin(heading_tuner)

    heading_tuner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
