from enum import Enum

class MissionType(Enum):
    NAVIGATION = 1
    GEOFENCE = 2

class Mission():
    def __init__(self):
        self.type = MissionType.NAVIGATION
        self.num_items = 0 # number of waypoints
        self.current_item = 0 # the waypoint being pursued
        self.waypoints = [] # list of mission waypoints



    
