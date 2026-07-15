import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached
from boat_iface.srv import ArmDisarm

from boat_control.Mission import Mission, MissionType

class VehicleSupervisor(Node):
    def __init__(self):
        super().__init__('vehicle_supervisor')
        
        ##### Vehicle Supervisor State #####
        self.armed = False
        ####################################

        self.mission_upload_srv = self.create_service(ArmDisarm, 'arm_disarm', self.arm_disarm_cb)
        
    def arm_disarm_cb(self, request, response):
        if request.arm == True:
            if self.armed == False:
                response.success = True
                response.err = 0
                response.message = 'success'
            else:
                response.success = False
                response.err = 1
                response.message = 'vehicle already armed'
        elif request.arm == False:
            if self.armed == False:
                response.success = False
                response.err = 2
                response.message = 'vehicle already disarmed'
            else:
                response.success = False
                response.err = 0
                response.message = 'success'

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