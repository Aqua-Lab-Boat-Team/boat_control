import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached
from boat_iface.srv import UploadMission

from boat_control.Mission import Mission, MissionType
from boat_control.enums.mission_state import MissionState

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.mission = Mission()
        self.state = MissionState.NO_MISSION
        self.mission_upload_srv = self.create_service(UploadMission, 'upload_mission', self.upload_mission_cb)

        self.timer = self.create_timer(0.01, self.loop)
        
    def upload_mission_cb(self, request, response):
        candidate_mission = self.mission_from_request(request)
        if self.validate_mission(candidate_mission):
            self.mission = candidate_mission
            self.state = MissionState.NOT_STARTED
            response.success = True
            self.get_logger().info(f"Mission Upload Success")
        else:
            response.success = False
            self.get_logger().error(f"Mission Upload Failure")

        return response

    def validate_mission(self, mission: Mission):
        return True

    def mission_from_request(self, request):
        temp_mission = Mission()
        waypoints = [item for item in request.items]
        num_items = len(waypoints)
        temp_mission.waypoints = waypoints
        temp_mission.num_items = num_items

        return temp_mission


    def loop(self):
        # Listen to publishers that may change the mission state

        match self.state:
            case MissionState.NO_MISSION:
                pass 
            case MissionState.NOT_STARTED:

                pass 
            case MissionState.ACTIVE:
                # Publish goal position
                pass
            case MissionState.PAUSED:
                pass
            case MissionState.COMPLETED:
                pass
            case MissionState.ABORTED:
                pass
            
def main(args=None):
    rclpy.init(args=args)

    mission_manager = MissionManager()

    rclpy.spin(mission_manager)

    mission_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()