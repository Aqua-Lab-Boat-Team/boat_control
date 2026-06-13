from rclpy.node import Node
from boat_iface.msg import MissionItemInt
from boat_control.MissionItem import MissionItem

class MissionItemPublisher(Node):
    def __init__(self):
        super().__init__('mission_item_publisher')

        self.publisher_ = self.create_publisher(MissionItemInt, '/mission_items', 20)

    def publish(self, item: MissionItem):
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

        self.publisher_.publish(msg)