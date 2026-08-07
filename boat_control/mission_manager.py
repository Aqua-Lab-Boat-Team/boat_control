import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from boat_iface.msg import MissionAck, MissionItemInt, MissionCount, MissionItemReached, GoalWaypoint, VehicleSupervisorState, GPS
from boat_iface.srv import UploadMission, FlightModeChange

from boat_control.Mission import Mission, MissionType
from boat_control.enums.mission_state import MissionState
from boat_control.enums.flight_mode import FlightMode
from boat_control.data.supervisor_state_cache import SupervisorStateCache
from boat_control.services.flight_mode_change_client import FlightModeChangeClient

from boat_control.helpers.math_helpers import distance_between_coordinates, goal_is_reached

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        ##### LOCAL STATE #####
        self.mission = Mission()
        self.state = MissionState.NO_MISSION
        self.cache = SupervisorStateCache()
        self.lat: int | None = None
        self.long: int | None = None
        #######################

        #### SERVERS ####
        self.mission_upload_srv = self.create_service(UploadMission, 'upload_mission', self.upload_mission_cb)
        #################

        #### CLIENTS ####
        self.flight_mode_change_client = FlightModeChangeClient()
        #################

        #### PUBLISHERS #####
        self.goal_waypoint_pub = self.create_publisher(GoalWaypoint, '/mission/goal_waypoint', 10)
        self.vehicle_supervisor_state_sub = self.create_subscription(VehicleSupervisorState, '/vehicle/vehicle_supervisor_state', self.supervisor_state_sub_cb, 10)
        #####################

        #### SUBSCRIBERS ####
        self.gps_sub = self.create_subscription(GPS, '/vehicle/sensors/gps', self.gps_sub_cb, qos_profile_sensor_data)
        #####################
        self.timer = self.create_timer(0.01, self.loop)
        
    def upload_mission_cb(self, request, response):
        candidate_mission = self.mission_from_request(request)
        if self.validate_mission(candidate_mission):
            self.mission = candidate_mission
            self.state = MissionState.NOT_STARTED
            response.success = True
            self.state = MissionState.NOT_STARTED
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
                if self.cache.arm_state:
                    response = self.request_guided_mode()
                    self.get_logger().info("Requesting Guided Mode...")
                    if response.success:
                        self.state = MissionState.ACTIVE
                        self.get_logger().info("Guided mode granted! Running mission...")
            case MissionState.ACTIVE:
                self.get_logger().info(f"arm: {self.cache.arm_state}, mode: {self.cache.flight_mode}")
                if self.cache.arm_state == True:
                    #if self.cache.flight_mode == FlightMode.GUIDED:
                    if (self.lat is not None) and (self.long is not None):
                        self.publish_goal_waypoint()
            case MissionState.PAUSED:
                pass
            case MissionState.COMPLETED:
                pass
            case MissionState.ABORTED:
                pass

    def publish_goal_waypoint(self):
        if self.mission.current_item >= self.mission.num_items:
            return

        current_waypoint = self.mission.waypoints[self.mission.current_item]
        goal_waypoint = GoalWaypoint()
        goal_waypoint.x = current_waypoint.x
        goal_waypoint.y = current_waypoint.y
        self.get_logger().info(f'{format(goal_waypoint.x / 1e7, ".12f")}')

        ### TEST ####
        self.get_logger().info(f'Distance: {distance_between_coordinates(goal_waypoint.x / 1e7, goal_waypoint.y / 1e7, self.lat, self.long)}')
        #############
        
        if goal_is_reached(self.lat, self.long, goal_waypoint.x / 1e7, goal_waypoint.y / 1e7) <= 10:
            if not (self.mission.current_item >= self.mission.num_items - 1):
                self.mission.current_item += 1

        self.goal_waypoint_pub.publish(goal_waypoint)

    def supervisor_state_sub_cb(self, msg):
        self.cache.arm_state = msg.armed
        self.cache.flight_mode = FlightMode(msg.flight_mode)

    def request_guided_mode(self):
        future = self.flight_mode_change_client.send_request(FlightMode.GUIDED)
        rclpy.spin_until_future_complete(self.flight_mode_change_client, future)
        response = future.result()
        return response

    def gps_sub_cb(self, msg) -> None:
        self.lat = msg.latitude
        self.long = msg.longitude

            
def main(args=None):
    rclpy.init(args=args)

    mission_manager = MissionManager()

    rclpy.spin(mission_manager)

    mission_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()