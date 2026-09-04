import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached, VehicleSupervisorState
from boat_iface.srv import ArmDisarm, FlightModeChange
from boat_control.core.mission.Mission import Mission, MissionType
from boat_control.enums.flight_mode import FlightMode

class VehicleSupervisor(Node):
    def __init__(self):
        super().__init__('vehicle_supervisor')
        
        ##### Vehicle Supervisor State #####
        self.armed = False
        self.flight_mode = FlightMode.HOLD
        ####################################


        self.vehicle_supervisor_state_pub = self.create_publisher(VehicleSupervisorState, '/vehicle/vehicle_supervisor_state', 10)
        self.vehicle_supervisor_state_pub_timer = self.create_timer(0.05, self.vehicle_supervisor_state_pub_cb)
        self.arm_disarm_srv = self.create_service(ArmDisarm, 'arm_disarm', self.arm_disarm_cb)
        self.flight_mode_change_srv = self.create_service(FlightModeChange, 'flight_mode_change', self.flight_mode_change_cb)
        
    def vehicle_supervisor_state_pub_cb(self):
        vehicle_supervisor_state = VehicleSupervisorState()
        vehicle_supervisor_state.armed = self.armed
        vehicle_supervisor_state.flight_mode = self.flight_mode.value
        self.vehicle_supervisor_state_pub.publish(vehicle_supervisor_state)
    
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
                response.success = True
                response.err = 0
                response.message = 'success'
                self.armed = False
                self.flight_mode = FlightMode.HOLD

        self.get_logger().info(f"RECV: {request}")
        return response

    def flight_mode_change_cb(self, request, response):
        self.get_logger().info(f"RECV: {request}")
        if self.armed:
            self.flight_mode = FlightMode(request.requested_flight_mode)
            response.success = True
            response.err = 0
            response.message = f"Mode changed to {FlightMode(request.requested_flight_mode)}"
        return response


def main(args=None):
    rclpy.init(args=args)

    vehicle_supervisor = VehicleSupervisor()

    rclpy.spin(vehicle_supervisor)

    vehicle_supervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()