from pymavlink import mavutil

class MissionItem:
    def __init__(self, seq, frame, command, current, autocontinue, x, y, z):
        self.seq = seq
        self.frame = frame
        self.command = command
        self.current = current
        self.autocontinue = autocontinue
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def message_to_mission_item(cls, _m:  mavutil.mavlink.MAVLink_message):
        if _m.get_type() != "MISSION_ITEM_INT":
            raise ValueError(f"Expected MISSION_ITEM_INT, got {_m.get_type()}")

        return cls(
            seq=_m.seq,
            frame=_m.frame,
            command=_m.command,
            current=bool(_m.current),
            autocontinue=bool(_m.autocontinue),
            x=_m.x,
            y=_m.y,
            z=_m.z,
        )
        