import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from boat_iface.msg import MissionItemInt

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MissionItemInt, 'mission_item_int', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = MissionItemInt()
        msg.seq = 16
        msg.frame = 2
        msg.command = 0
        msg.current = 6
        msg.autocontinue = 1
        msg.param1 = 1.0
        msg.param2 = 2.0
        msg.param3 = 3.0
        msg.param4 = 4.0
        msg.x = 145
        msg.y = 198
        msg.z = 225
        msg.mission_type = 3

        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
