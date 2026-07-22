from rclpy.node import Node
from boat_iface.msg import MissionItemInt
from boat_iface.srv import UploadMission
from boat_iface.srv import FlightModeChange

class FlightModeChangeClient(Node):
    def __init__(self):
        super().__init__('flight_mode_change_client')
        self.cli = self.create_client(FlightModeChange, 'flight_mode_change')
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Flight mode change not currently available. Retrying...')

    def send_request(self, mode:int):
        request = FlightModeChange.Request()      # Create a request

        request.request_arm = arm
        future = self.cli.call_async(request)  # Make async request

        return future