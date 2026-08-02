#!/usr/bin/env python3

import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from pyubx2 import UBXReader, UBX_PROTOCOL, NMEA_PROTOCOL


class UbloxGpsNode(Node):
    def __init__(self) -> None:
        super().__init__("ublox_gps")

        # Change these through ROS parameters when launching the node.
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 38400)

        # "relpos": dual-antenna moving-base heading from NAV-RELPOSNED
        # "motion": course over ground from NAV-PVT
        self.declare_parameter("heading_mode", "relpos")

        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        self.heading_mode = str(
            self.get_parameter("heading_mode").value
        ).lower()

        if self.heading_mode not in ("relpos", "motion"):
            raise ValueError(
                "heading_mode must be either 'relpos' or 'motion'"
            )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/vehicle/sensors/gps",
            qos_profile_sensor_data,
        )

        self.heading_deg: float | None = None

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
                f"Could not open GPS UART {port}: {error}"
            )
            raise

        # Parse only binary UBX messages.
        self.ubx_reader = UBXReader(
            self.serial_port,
            protfilter=NMEA_PROTOCOL | UBX_PROTOCOL,
        )

        # Read available UART messages without permanently blocking ROS.
        self.timer = self.create_timer(0.01, self.read_uart)

        self.get_logger().info(
            f"Reading ZED-F9P from {port} at {baudrate} baud; "
            f"heading mode: {self.heading_mode}"
        )

    def read_uart(self) -> None:
        try:
            # Process several buffered messages per callback so that the
            # serial receive buffer does not fall behind.
            for _ in range(10):
                if self.serial_port.in_waiting == 0:
                    break

                _, ubx_message = self.ubx_reader.read()

                if ubx_message is None:
                    continue
                self.handle_ubx_message(ubx_message)

        except serial.SerialException as error:
            self.get_logger().error(f"GPS UART error: {error}")
        except Exception as error:
            self.get_logger().warning(f"Could not parse GPS data: {error}")

    def handle_ubx_message(self, ubx_message) -> None:
        if ubx_message.identity == "NAV-RELPOSNED":
            self.handle_relposned(ubx_message)

        elif ubx_message.identity == "NAV-PVT":
            self.handle_nav_pvt(ubx_message)

        elif ubx_message.identity == "GNGLL":
            self.handle_gngll(ubx_message)

    def handle_relposned(self, ubx_message) -> None:
        if self.heading_mode != "relpos":
            return

        heading_valid = bool(
            getattr(ubx_message, "relPosHeadingValid", 0)
        )
        position_valid = bool(
            getattr(ubx_message, "relPosValid", 0)
        )

        if not heading_valid or not position_valid:
            self.heading_deg = None
            return

        self.heading_deg = float(ubx_message.relPosHeading) % 360.0

    def handle_nav_pvt(self, ubx_message) -> None:
        fix_valid = bool(getattr(ubx_message, "gnssFixOk", 0))
        coordinates_invalid = bool(
            getattr(ubx_message, "invalidLlh", 0)
        )

        if not fix_valid or coordinates_invalid:
            return

        latitude = float(ubx_message.lat)
        longitude = float(ubx_message.lon)

        if self.heading_mode == "motion":
            # Course over ground. This is not reliable while stationary.
            self.heading_deg = float(ubx_message.headMot) % 360.0

        # In relpos mode, wait until a valid NAV-RELPOSNED heading exists.
        if self.heading_deg is None:
            return

        output = Float64MultiArray()
        output.data = [
            latitude,
            longitude,
            self.heading_deg,
        ]

        self.publisher.publish(output)

    def handle_gngll(self, ubx_message) -> None:
        # GLL status:
        # "A" = valid position
        # "V" = invalid position
        if getattr(ubx_message, "status", "V") != "A":
            self.get_logger().debug("GNGLL position is not valid")
            return

        try:
            latitude = float(ubx_message.lat)
            longitude = float(ubx_message.lon)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(
                f"Invalid GNGLL coordinates: {error}"
            )
            return

        output = Float64MultiArray()
        output.data = [
            latitude,
            longitude,
        ]

        self.publisher.publish(output)

        self.get_logger().debug(
            f"Published latitude={latitude:.8f}, "
            f"longitude={longitude:.8f}"
        )



    def destroy_node(self) -> None:
        if hasattr(self, "serial_port") and self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UbloxGpsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()