from rclpy.node import Node
from boat_control.MissionItem import MissionItem
from boat_iface.msg import MissionItemInt
from boat_iface.srv import UploadMission


class MissionUploadClient(Node):
    def __init__(self):
        super().__init__('mission_upload_client')
        self.cli = self.create_client(UploadMission, 'upload_mission')
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Mission upload not available, waiting again...')


    def mission_item_to_msg(self, item: MissionItem):
        msg = MissionItemInt()

        msg.seq = item.seq
        msg.frame = item.frame
        msg.command = item.command
        msg.current = item.current
        msg.autocontinue = item.autocontinue
        msg.param1 = item.param1
        msg.param2 = item.param2
        msg.param3 = item.param3
        msg.param4 = item.param4
        msg.x = item.x
        msg.y = item.y
        msg.z = item.z

        return msg

    def send_request(self, items: list[MissionItem]):
        request = UploadMission.Request()      # Create a request

        request.items = [                      # Convert each mission item to MissionItemInt type
            self.mission_item_to_msg(item)
            for item in items
        ]
        future = self.cli.call_async(request)  # Make async request

        return future