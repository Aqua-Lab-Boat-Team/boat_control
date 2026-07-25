from rclpy.node import Node

from boat_iface.srv import FlightModeChange
from boat_control.enums.flight_mode import FlightMode

class FlightModeChangeClient(Node):
    def __init__(self):
        super().__init__('flight_mode_change_client')
        self.cli = self.create_client(FlightModeChange, 'flight_mode_change')
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Flight mode change not currently available. Retrying...')

    def send_request(self, mode:FlightMode):
        request = FlightModeChange.Request()      # Create a request

        request.requested_flight_mode = mode.value
        future = self.cli.call_async(request)  # Make async request

        return future