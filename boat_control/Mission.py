
class MissionType(Enum):
    NAVIGATION = 1
    GEOFENCE = 2

class Mission():
    def __init__(self):
        self.type = MissionType.NAVIGATION
        self.num_items = 0 # number of waypoints
        self.current_item = 0 # the waypoint being pursued
        self.upload_complete = False # mission upload complete
        self.mission_active = False
        self.last_item_received = 0 # number of last item received in upload process
        self.waypoints = [] # list of mission waypoints



    
