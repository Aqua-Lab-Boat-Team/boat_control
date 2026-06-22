import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached
from boat_iface.srv import UploadMission

from boat_control.Mission import Mission, MissionType

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # self.subs = []
        # self.topics = [
        #     (MissionItemInt, '/mission_items', self.mission_item_int_cb, 10),
        # ]

        self.mission_upload_srv = self.create_service(UploadMission, 'upload_mission', self.upload_mission_cb)
        
    def upload_mission_cb(self, request, response):
        response.success = True
        self.get_logger().info(f"RECV: {request}")
        return response


def main(args=None):
    rclpy.init(args=args)

    mission_manager = MissionManager()

    rclpy.spin(mission_manager)

    mission_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()