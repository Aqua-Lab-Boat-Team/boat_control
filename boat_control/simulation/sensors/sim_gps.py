import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from boat_control.enums.flight_mode import FlightMode
from boat_control.helpers.math_helpers import distance_between_coordinates
from boat_iface.msg import GPS, MissionItems, VehicleSupervisorState


class SimGpsNode(Node):
    def __init__(self) -> None:
        super().__init__('sim_gps')

        self.declare_parameter('speed_mps', 1.0)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.speed_mps = float(self.get_parameter('speed_mps').value)
        update_rate_hz = float(
            self.get_parameter('update_rate_hz').value
        )
        publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )

        if self.speed_mps < 0.0:
            raise ValueError('speed_mps must be non-negative')
        if update_rate_hz <= 0.0:
            raise ValueError('update_rate_hz must be greater than zero')
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')

        self.mission_items = []
        self.target_index = 0
        self.latitude: float | None = None
        self.longitude: float | None = None
        self.heading = 0.0
        self.vx = 0
        self.vy = 0
        self.armed = False
        self.flight_mode = FlightMode.HOLD.value
        self.last_update_ns = self.get_clock().now().nanoseconds

        self.gps_pub = self.create_publisher(
            GPS,
            '/vehicle/sensors/gps',
            qos_profile_sensor_data,
        )
        self.mission_items_sub = self.create_subscription(
            MissionItems,
            '/simulation/mission_items',
            self.mission_items_cb,
            10,
        )
        self.supervisor_state_sub = self.create_subscription(
            VehicleSupervisorState,
            '/vehicle/vehicle_supervisor_state',
            self.supervisor_state_cb,
            10,
        )

        self.update_timer = self.create_timer(
            1.0 / update_rate_hz,
            self.update_position,
        )
        self.publish_timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_gps,
        )

    def mission_items_cb(self, msg: MissionItems) -> None:
        self.mission_items.clear()
        self.mission_items.extend(msg.items)
        self.target_index = 0
        self.vx = 0
        self.vy = 0
        self.heading = 0.0
        self.last_update_ns = self.get_clock().now().nanoseconds

        if not self.mission_items:
            self.latitude = None
            self.longitude = None
            return

        first_item = self.mission_items[0]
        self.latitude = first_item.x / 1e7
        self.longitude = first_item.y / 1e7
        self.target_index = 1

    def supervisor_state_cb(self, msg: VehicleSupervisorState) -> None:
        self.armed = msg.armed
        self.flight_mode = msg.flight_mode

        if not self.can_move():
            self.vx = 0
            self.vy = 0

    def can_move(self) -> bool:
        return (
            self.armed
            and self.flight_mode == FlightMode.GUIDED.value
            and self.speed_mps > 0.0
        )

    def update_position(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = max(0.0, (now_ns - self.last_update_ns) / 1e9)
        self.last_update_ns = now_ns

        if (
            not self.can_move()
            or self.latitude is None
            or self.longitude is None
            or self.target_index >= len(self.mission_items)
        ):
            self.vx = 0
            self.vy = 0
            return

        distance_to_travel_m = self.speed_mps * elapsed_s

        while (
            distance_to_travel_m > 0.0
            and self.target_index < len(self.mission_items)
        ):
            target = self.mission_items[self.target_index]
            target_latitude = target.x / 1e7
            target_longitude = target.y / 1e7
            segment_distance_m = distance_between_coordinates(
                self.latitude,
                self.longitude,
                target_latitude,
                target_longitude,
            )

            if segment_distance_m == 0.0:
                self.latitude = target_latitude
                self.longitude = target_longitude
                self.target_index += 1
                continue

            self.heading = self.bearing_degrees(
                self.latitude,
                self.longitude,
                target_latitude,
                target_longitude,
            )

            if distance_to_travel_m >= segment_distance_m:
                self.latitude = target_latitude
                self.longitude = target_longitude
                self.target_index += 1
                distance_to_travel_m -= segment_distance_m
                continue

            fraction = distance_to_travel_m / segment_distance_m
            self.latitude += (
                target_latitude - self.latitude
            ) * fraction
            self.longitude += (
                target_longitude - self.longitude
            ) * fraction
            distance_to_travel_m = 0.0

        if self.target_index >= len(self.mission_items):
            self.vx = 0
            self.vy = 0
        else:
            heading_radians = math.radians(self.heading)
            self.vx = round(
                self.speed_mps * math.cos(heading_radians) * 100.0
            )
            self.vy = round(
                self.speed_mps * math.sin(heading_radians) * 100.0
            )

    @staticmethod
    def bearing_degrees(
        start_latitude: float,
        start_longitude: float,
        target_latitude: float,
        target_longitude: float,
    ) -> float:
        start_latitude_radians = math.radians(start_latitude)
        target_latitude_radians = math.radians(target_latitude)
        longitude_delta_radians = math.radians(
            target_longitude - start_longitude
        )

        east = (
            math.sin(longitude_delta_radians)
            * math.cos(target_latitude_radians)
        )
        north = (
            math.cos(start_latitude_radians)
            * math.sin(target_latitude_radians)
            - math.sin(start_latitude_radians)
            * math.cos(target_latitude_radians)
            * math.cos(longitude_delta_radians)
        )
        return math.degrees(math.atan2(east, north)) % 360.0

    def publish_gps(self) -> None:
        if self.latitude is None or self.longitude is None:
            return

        msg = GPS()
        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.heading = self.heading
        msg.vx = self.vx
        msg.vy = self.vy
        msg.vz = 0
        self.gps_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimGpsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
