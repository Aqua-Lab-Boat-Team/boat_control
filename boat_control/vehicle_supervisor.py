import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached
from boat_iface.srv import ArmDisarm
from boat_iface.msg import ArmState

from boat_control.Mission import Mission, MissionType

class VehicleSupervisor(Node):
    def __init__(self):
        super().__init__('vehicle_supervisor')
        
        ##### Vehicle Supervisor State #####
        self.armed = False
        ####################################
        self.arm_state_pub = self.create_publisher(ArmState, '/vehicle/arm_state', 10)
        self.arm_state_pub_timer = self.create_timer(0.5, self.arm_pub_cb)
        self.mission_upload_srv = self.create_service(ArmDisarm, 'arm_disarm', self.arm_disarm_cb)
        
    def arm_pub_cb(self):
        arm_state = ArmState()
        arm_state.armed = self.armed
        self.arm_state_pub.publish(arm_state)
    
    def arm_disarm_cb(self, request, response):
        if request.request_arm == True:
            if self.armed == False:
                response.success = True
                response.err = 0
                response.message = 'success'
                self.armed = True
            else:
                response.success = False
                response.err = 1
                response.message = 'vehicle already armed'
        elif request.request_arm == False:
            if self.armed == False:
                response.success = False
                response.err = 2
                response.message = 'vehicle already disarmed'
            else:
                response.success = False
                response.err = 0
                response.message = 'success'
                self.armed = False

        self.get_logger().info(f"RECV: {request}")
        return response


def main(args=None):
    rclpy.init(args=args)

    vehicle_supervisor = VehicleSupervisor()

    rclpy.spin(vehicle_supervisor)

    vehicle_supervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()