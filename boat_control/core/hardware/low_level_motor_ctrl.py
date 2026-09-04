import serial, struct
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MotorPower
from boat_control.helpers.math_helpers import *
from boat_control.data.comms_config import CommsConfig


class LowLevelMotorControl(Node):
    def __init__(self):
        super().__init__('low_level_motor_ctrl')
        
        self.declare_parameter("port", CommsConfig.ARDUINO_UART_PORT)
        self.declare_parameter("baudrate", CommsConfig.ARDUINO_UART_BAUD)
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
                f"Could not open low level motor controller {port}: {error}"
            )
            raise

        self.get_logger().info(
            f"Arduino UART {port} at {baudrate} baud"
        )

        self.motor_power_subscriber = self.create_subscription(MotorPower, '/vehicle/motor_power', motor_power_cb, 10)

    def motor_power_cb(self, msg):
        # Collect from message
        left_power = msg.left_power
        right_power = msg.right_power
        
        # Artificially capping min and max thruster power
        left_power = map(left_power, -1, 1, -0.75, 0.75)
        right_power = map(right_power, -1, 1, -0.75, 0.75)
        
        # Mapping to range accepted by external controller
        left_power = int_map(left_power, -1, 1, -128, 127)
        right_power = int_map(right_power, -1, 1, -128, 127)
        
        command = f"{left_power} {right_power}\n"
        self.get_logger().debug(command)
        self.write_to_controller(command)

    def write_to_controller(self, command: str):
        self.serial_port.write(command.encode("ascii"))

    def destroy_node(self) -> None:
        if hasattr(self, "serial_port") and self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    low_level_motor_ctrl = LowLevelMotorControl()

    rclpy.spin(low_level_motor_ctrl)

    low_level_motor_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
