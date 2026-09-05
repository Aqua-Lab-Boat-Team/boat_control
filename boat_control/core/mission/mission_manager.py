import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from boat_iface.msg import GoalWaypoint, GPS, MissionItems, VehicleSupervisorState
from boat_iface.srv import UploadMission, FlightModeChange

from boat_control.core.mission.Mission import Mission, MissionType
from boat_control.enums.mission_state import MissionState
from boat_control.enums.flight_mode import FlightMode
from boat_control.data.supervisor_state_cache import SupervisorStateCache
from boat_control.core.services.flight_mode_change_client import FlightModeChangeClient

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

        #### PARAMETERS ####
        self.declare_parameter('use_sim_gps', False)
        self.use_sim_gps = self.get_parameter(
            'use_sim_gps'
        ).get_parameter_value().bool_value
        ####################

        #### SERVERS ####
        self.mission_upload_srv = self.create_service(UploadMission, 'upload_mission', self.upload_mission_cb)
        #################

        #### CLIENTS ####
        self.flight_mode_change_client = FlightModeChangeClient(self)
        #################

        #### PUBLISHERS #####
        self.goal_waypoint_pub = self.create_publisher(GoalWaypoint, '/mission/goal_waypoint', 10)
        self.sim_mission_items_pub = None
        if self.use_sim_gps:
            self.sim_mission_items_pub = self.create_publisher(
                MissionItems, '/simulation/mission_items', 10
            )
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
            if self.sim_mission_items_pub is not None:
                mission_items = MissionItems()
                mission_items.items = request.items
                self.sim_mission_items_pub.publish(mission_items)
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
                if self.cache.arm_state and self.cache.flight_mode != FlightMode.MANUAL:
                    self.get_logger().info("Requesting Guided Mode...")
                    self.request_guided_mode()
            case MissionState.ACTIVE:
                if self.cache.arm_state == True:
                    if self.cache.flight_mode == FlightMode.GUIDED:
                        if (self.lat is not None) and (self.long is not None):
                            self.publish_goal_waypoint()
                    else:
                        self.state = MissionState.PAUSED
            case MissionState.PAUSED:
                if self.cache.flight_mode == FlightMode.GUIDED:
                    self.state = MissionState.ACTIVE
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
        # self.get_logger().info(f'WAYPOINT: {self.mission.current_item}')
        #self.get_logger().info(f'GOAL LAT: {format(goal_waypoint.x / 1e7, ".12f")}, LON: {format(goal_waypoint.y / 1e7, ".12f")} ')

        ### TEST ####
        # self.get_logger().info(f'Distance: {distance_between_coordinates(goal_waypoint.x / 1e7, goal_waypoint.y / 1e7, self.lat, self.long)}')
        #############
        
        if goal_is_reached(self.lat, self.long, goal_waypoint.x / 1e7, goal_waypoint.y / 1e7, 2):
            if not (self.mission.current_item >= self.mission.num_items - 1):
                self.mission.current_item += 1

        self.goal_waypoint_pub.publish(goal_waypoint)

    def supervisor_state_sub_cb(self, msg):
        self.cache.arm_state = msg.armed
        self.cache.flight_mode = FlightMode(msg.flight_mode)

    def request_guided_mode(self):
        future = self.flight_mode_change_client.send_request(FlightMode.GUIDED)
        future.add_done_callback(self.request_guide_mode_cb)

    def request_guide_mode_cb(self, future):
        response = future.result()
        if response.success:
            self.state = MissionState.ACTIVE
            self.get_logger().info("Guided mode granted! Running mission...")

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
