import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached
from Mission import Mission, MissionType

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.subscriptions = []
        self.topics = [
            (MissionItemInt, '/mission_item_int', self.mission_item_int_cb, 10),
        ]

        self.subscription = self.create_subscription(
            MissionItemInt,
            '/mission_item_int',
            self.mission_item_int_cb,
            10)

    def mission_item_int_cb(self, msg: MissionItemInt):
        self.get_logger().info(f'INT: {msg.seq}')

def main(args=None):
    rclpy.init(args=args)

    mission_manager = MissionManager()

    rclpy.spin(mission_manager)

    mission_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()