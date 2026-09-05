from rclpy.node import Node
from boat_iface.msg import MissionItemInt
from boat_iface.srv import UploadMission
from boat_iface.srv import ArmDisarm

class ArmDisarmClient():
    def __init__(self, node):
        self.node = node
        self.cli = self.node.create_client(ArmDisarm, 'arm_disarm')
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().info('Arm/Disarm not currently available. Retrying...')

    def send_request(self, arm:bool):
        request = ArmDisarm.Request()      # Create a request

        request.request_arm = arm
        future = self.cli.call_async(request)  # Make async request

        return future