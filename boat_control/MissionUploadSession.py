from boat_control.MissionItem import MissionItem

class MissionUploadSession:
    def __init__(self):
        self.mission_type = 0
        self.upload_active = False
        self.num_mission_items = 0
        self.last_rec_item = -1
        self.request_timeout_ms = 100
        self.max_retry = 5
        self.retry_count = 0
        self.is_waiting = False
        self.t_last_transmit = 0
        self.mission_item_list = []

    def add_mission_item(self, mission_item: MissionItem):
        self.retry_count = 0
        self.is_waiting = False
        self.last_rec_item += 1
        self.mission_item_list.append(mission_item)

    def received_all_items(self):
        return self.last_rec_item == self.num_mission_items - 1
    
    def reset(self):
        self.nums_mission_items = 0
        self.current_mission_item = 0
        self.retry_count = 0
        self.is_waiting = False
        self.t_last_transmit = -1
        self.mission_item_list = []

    
