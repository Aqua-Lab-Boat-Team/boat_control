from rclpy.node import Node

from boat_iface.srv import FlightModeChange
from boat_control.enums.flight_mode import FlightMode

class FlightModeChangeClient():
    def __init__(self, node):
        self.node = node
        self.cli = self.node.create_client(FlightModeChange, 'flight_mode_change')
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Flight mode change not currently available. Retrying...')

    def send_request(self, mode:FlightMode):
        request = FlightModeChange.Request()      # Create a request

        request.requested_flight_mode = mode.value
        future = self.cli.call_async(request)  # Make async request

        return future