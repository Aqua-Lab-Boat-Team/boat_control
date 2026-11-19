"""Simulate GPS feedback from a pair of differential-drive thrusters."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from boat_control.enums.flight_mode import FlightMode
from boat_control.helpers.math_helpers import EARTH_RADIUS_M, clamp
from boat_iface.msg import GPS, MissionItems, MotorPower, VehicleSupervisorState


class SimBoatNode(Node):
    """Integrate a planar boat pose and publish it as a GPS fix."""

    def __init__(self) -> None:
        super().__init__('sim_boat')

        self.origin_latitude = self.declare_parameter('initial_latitude', 0.0).value
        self.origin_longitude = self.declare_parameter('initial_longitude', 0.0).value
        initial_heading = self.declare_parameter('initial_heading_deg', 0.0).value
        self.full_power_speed_mps = self.declare_parameter(
            'full_power_speed_mps', 1.0
        ).value
        self.thruster_spacing_m = self.declare_parameter(
            'thruster_spacing_m', 1.0
        ).value
        publish_rate_hz = self.declare_parameter('publish_rate_hz', 10.0).value
        respect_supervisor = self.declare_parameter('respect_supervisor', False).value
        self.mission_origin_pending = self.declare_parameter(
            'initialize_from_mission', False
        ).value

        if not -90.0 <= self.origin_latitude <= 90.0:
            raise ValueError('initial_latitude must be between -90 and 90 degrees')
        if not -180.0 <= self.origin_longitude <= 180.0:
            raise ValueError('initial_longitude must be between -180 and 180 degrees')
        if not math.isfinite(initial_heading):
            raise ValueError('initial_heading_deg must be finite')
        if not math.isfinite(self.full_power_speed_mps) or self.full_power_speed_mps < 0.0:
            raise ValueError('full_power_speed_mps must be finite and non-negative')
        if not math.isfinite(self.thruster_spacing_m) or self.thruster_spacing_m <= 0.0:
            raise ValueError('thruster_spacing_m must be finite and positive')
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be finite and positive')

        self.north_m = 0.0
        self.east_m = 0.0
        self.heading_rad = math.radians(initial_heading % 360.0)
        self.forward_speed_mps = 0.0
        self.heading_rate_rad_s = 0.0
        self.motors_enabled = not respect_supervisor
        self.last_update_ns = self.get_clock().now().nanoseconds

        self.gps_pub = self.create_publisher(
            GPS, '/vehicle/sensors/gps', qos_profile_sensor_data
        )
        self.motor_power_sub = self.create_subscription(
            MotorPower, '/vehicle/motor_power', self.motor_power_cb, 10
        )
        if respect_supervisor:
            self.supervisor_state_sub = self.create_subscription(
                VehicleSupervisorState,
                '/vehicle/vehicle_supervisor_state',
                self.supervisor_state_cb,
                10,
            )
        if self.mission_origin_pending:
            self.mission_items_sub = self.create_subscription(
                MissionItems, '/simulation/mission_items', self.mission_items_cb, 10
            )
        self.publish_timer = self.create_timer(1.0 / publish_rate_hz, self.publish_gps)

    def motor_power_cb(self, msg: MotorPower) -> None:
        """Apply a new command after integrating the previous one to this time."""
        if not self.motors_enabled:
            return

        self.update_position()
        if not (math.isfinite(msg.left_power) and math.isfinite(msg.right_power)):
            self.forward_speed_mps = 0.0
            self.heading_rate_rad_s = 0.0
            return

        # Match the low-level motor controller's 75 percent output scaling.
        speed_scale = 0.75 * self.full_power_speed_mps
        left_speed = clamp(msg.left_power, -1.0, 1.0) * speed_scale
        right_speed = clamp(msg.right_power, -1.0, 1.0) * speed_scale
        self.forward_speed_mps = (left_speed + right_speed) / 2.0
        # A faster left thruster turns the bow right, increasing compass heading.
        self.heading_rate_rad_s = (left_speed - right_speed) / self.thruster_spacing_m

    def supervisor_state_cb(self, msg: VehicleSupervisorState) -> None:
        """Clear the held command when the vehicle leaves a driving mode."""
        motors_enabled = msg.armed and msg.flight_mode in (
            FlightMode.MANUAL.value,
            FlightMode.GUIDED.value,
        )
        if self.motors_enabled and not motors_enabled:
            self.update_position()
            self.forward_speed_mps = 0.0
            self.heading_rate_rad_s = 0.0
        self.motors_enabled = motors_enabled

    def mission_items_cb(self, msg: MissionItems) -> None:
        """Use the first mission waypoint as the origin before the boat moves."""
        if not self.mission_origin_pending or not msg.items:
            return

        self.update_position()
        if not self.mission_origin_pending:
            return

        latitude = msg.items[0].x / 1e7
        longitude = msg.items[0].y / 1e7
        if -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0:
            self.origin_latitude = latitude
            self.origin_longitude = longitude
            self.mission_origin_pending = False

    def update_position(self) -> None:
        """Advance the pose along the arc of the current thruster command."""
        now_ns = self.get_clock().now().nanoseconds
        elapsed_s = (now_ns - self.last_update_ns) / 1e9
        self.last_update_ns = now_ns
        if elapsed_s <= 0.0:
            return
        if self.forward_speed_mps == 0.0 and self.heading_rate_rad_s == 0.0:
            return

        self.mission_origin_pending = False
        turn_rad = self.heading_rate_rad_s * elapsed_s
        half_turn_rad = turn_rad / 2.0
        midpoint_heading = self.heading_rad + half_turn_rad

        # Integrate the straight-line case and the circular arc with the same
        # midpoint heading. The sine ratio converts arc length to chord length.
        distance_m = self.forward_speed_mps * elapsed_s
        if half_turn_rad != 0.0:
            distance_m *= math.sin(half_turn_rad) / half_turn_rad
        self.north_m += distance_m * math.cos(midpoint_heading)
        self.east_m += distance_m * math.sin(midpoint_heading)
        self.heading_rad = (self.heading_rad + turn_rad) % math.tau

    def gps_coordinates(self) -> tuple[float, float]:
        """Project local north and east offsets to latitude and longitude."""
        distance_m = math.hypot(self.north_m, self.east_m)
        if distance_m == 0.0:
            return self.origin_latitude, self.origin_longitude

        origin_latitude_rad = math.radians(self.origin_latitude)
        bearing_rad = math.atan2(self.east_m, self.north_m)
        distance_rad = distance_m / EARTH_RADIUS_M
        sin_origin_latitude = math.sin(origin_latitude_rad)
        cos_origin_latitude = math.cos(origin_latitude_rad)
        sin_distance = math.sin(distance_rad)
        cos_distance = math.cos(distance_rad)

        north = sin_distance * math.cos(bearing_rad)
        east = sin_distance * math.sin(bearing_rad)
        # Rotate the unit vector into the origin's meridian before adding longitude.
        horizontal = cos_origin_latitude * cos_distance - sin_origin_latitude * north
        vertical = sin_origin_latitude * cos_distance + cos_origin_latitude * north
        latitude_rad = math.atan2(vertical, math.hypot(horizontal, east))
        longitude_delta_rad = math.atan2(east, horizontal)
        longitude = self.origin_longitude + math.degrees(longitude_delta_rad)
        return math.degrees(latitude_rad), (longitude + 180.0) % 360.0 - 180.0

    def publish_gps(self) -> None:
        """Publish compass heading and north/east velocity in cm/s."""
        self.update_position()
        msg = GPS()
        msg.latitude, msg.longitude = self.gps_coordinates()
        msg.heading = math.degrees(self.heading_rad) % 360.0
        speed_cm_s = self.forward_speed_mps * 100.0
        msg.vx = round(clamp(speed_cm_s * math.cos(self.heading_rad), -32768, 32767))
        msg.vy = round(clamp(speed_cm_s * math.sin(self.heading_rad), -32768, 32767))
        msg.vz = 0
        self.gps_pub.publish(msg)


def main(args=None) -> None:
    """Run the simulator node."""
    rclpy.init(args=args)
    node = SimBoatNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
