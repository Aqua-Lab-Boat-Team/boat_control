#!/usr/bin/env python3
"""
barebones_mavlink_ros2.py

Purpose:
- Keep your existing MAVLink script working
- Add a simple ROS 2 publisher
- Publish every MAVLink message received from QGC on a ROS 2 topic: /mavtopic

This is meant to be the FIRST step only.
No action server yet.
No custom ROS messages yet.
Just simple publishing so you can verify MAVLink -> ROS 2 flow.
"""

# Standard Python module for time handling
import time

# ROS 2 Python library
import rclpy
from builtins import getattr

# Base class for creating a ROS 2 node in Python
from rclpy.node import Node

# Message Type Imports
from std_msgs.msg import String
from boat_iface.msg import MissionItemInt

# Service Type Imports
from boat_iface.srv import UploadMission

# pymavlink helper library for MAVLink communication
from pymavlink import mavutil

# Local Imports
from boat_control.MissionUploadSession import MissionUploadSession
from boat_control.MissionItem import MissionItem
from boat_control.mission_upload_client import MissionUploadClient
from boat_control.data.comms_config import CommsConfig

# ============================================================
#                      GLOBAL STATE
# ============================================================
mvl_armed = False
mission_upload_active = False
mission_upload_sess = MissionUploadSession()
sys_stat_count = 0

def now_s() -> float:
    """
    Returns monotonic time in seconds.
    Monotonic means it always moves forward and is safe for interval timing.
    """
    return time.monotonic()


def millis() -> int:
    """
    Returns monotonic time in milliseconds.
    Useful for MAVLink messages like ATTITUDE_QUATERNION.
    """
    return int(now_s() * 1000)
# ======================================

class GCSInterface(Node):
    def __init__(self):
        super().__init__('gcs_interface')
        self.comms_config = CommsConfig()           # Create communications configuration. TODO: Accept parameters
        self.master = self.establish_gcs_connection() # Establish communication with the GCS station
        self.message_handlers = {
            mavutil.mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL:
                self.handle_manual_control,

            mavutil.mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                self.handle_param_request_list,

            mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG:
                self.handle_command_long,

            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                self.handle_mission_request_list,

            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_INT:
                self.handle_mission_item_int,

            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_COUNT:
                self.handle_mission_count,
        }
        self.mission_upload_client = MissionUploadClient()

        self.t_last_hb = now_s()
        self.t_last_sys = now_s()
        self.timer = self.create_timer(0.01, self.loop)

    def establish_gcs_connection(self):
        if not self.comms_config.USE_UDP:
            return mavutil.mavlink_connection(
                self.comms_config.UART_PORT,
                baud=self.comms_config.UART_BAUD,
                source_system=self.comms_config.MVL_SYSID,
                source_component=self.comms_configMVL_COMPID
            )
        else:
            print("UDP")
            return mavutil.mavlink_connection (
                self.comms_config.UDP_PORT,
                source_system=self.comms_config.MVL_SYSID,
                source_component=self.comms_config.MVL_COMPID
            )

    def loop(self):
        global mission_upload_sess
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is not None:
                print(msg)
                self.handle_mavlink_message(msg)

             # Send heartbeat periodically
            t = now_s()
 
            if (t - self.t_last_hb) >= self.comms_config.HB_INTERVAL:
                self.send_heartbeat(self.master)
                self.t_last_hb = t

            # Send status + attitude periodically
            if (t - self.t_last_sys) >= self.comms_config.SYS_STAT_INTERVAL:
                self.send_sys_status_and_att(self.master)
                self.t_last_sys = t

            # Retry message transmit
            if (mission_upload_sess.is_waiting):
                if (millis() - mission_upload_sess.t_last_transmit > mission_upload_sess.request_timeout_ms):
                    if (mission_upload_sess.retry_count < mission_upload_sess.max_retry):
                        print("RETRANSMIT")
                        self.send_mission_request_int(self.master)
                        mission_upload_sess.retry_count += 1
                    else:
                        print("MISSION UPLOAD TIMEOUT")


    def handle_mavlink_message(self, msg):
        msg_id = msg.get_msgId()
        handler = self.message_handlers.get(msg_id)

        if handler is None:
            self.get_logger().debug(f"Unhandled MAVLink message ID: {msg_id}")
            return
        
        if msg.get_srcSystem() == self.comms_config.GCS_SYSID:
            handler(msg, self.master)

    def handle_manual_control(self, m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
        master.mav.manual_control_send(
            target=m.target,
            x=m.x,
            y=m.y,
            z=m.z,
            r=m.r,
            buttons=m.buttons
        )


    def handle_param_request_list(self, _m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
        """
        Handles PARAM_REQUEST_LIST from QGC.

        Here we reply with a single dummy parameter.
        This helps QGC see that the MAVLink endpoint is alive and responding.
        """
        param_id = b"a_parm"

        # master.mav.param_value_send(
        #     param_id=param_id,
        #     param_value=123.456,
        #     param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        #     param_count=1,
        #     param_index=0,
        #     cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        #     mvl_armed = (int(m.param1) == 1)
        # )


    def handle_mission_request_list(self, _m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
        """
        Handles MISSION_REQUEST_LIST from QGC.

        For now, this sends a request for mission item index 0.
        This keeps your original behavior.
        """
        # master.mav.mission_request_int_send(
        #     target_system=MVL_SYSID,
        #     target_component=MVL_COMPID,
        #     seq=0
        # )


    def handle_mission_count(self, _m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
        """
        Handles MISSION_COUNT from QGC.

        This tells us how many mission items QGC wants to send.
        We store the count, mark upload as active, then request the first item.
        """
        global mission_upload_active
        global mission_upload_sess

        print(_m)

        if not mission_upload_active:
            mission_upload_active = True
            mission_upload_sess.num_mission_items = _m.count
            print(f"# Mission items: {mission_upload_sess.num_mission_items}")

        self.send_mission_request_int(_m, master)


    def handle_mission_item_int(self, _m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile):
        """
        Handles one MISSION_ITEM_INT from QGC.
        """

        global mission_upload_active
        global mission_upload_sess

        # Process the mission item
        mission_upload_sess.retry_count = 0
        mission_upload_sess.is_waiting = False
        mission_item = MissionItem.message_to_mission_item(_m) # Parse the mission item into an object
        mission_upload_sess.add_mission_item(mission_item) # Add the mission item to the current list

        # If we haven't seen everything yet, ask for the next item
        if mission_upload_sess.current_mission_item < mission_upload_sess.num_mission_items - 1:
            mission_upload_sess.current_mission_item += 1
            self.send_mission_request_int(_m, master)
            
        # If we've seen everything, acknowledge the mission and process it
        else:
            self.send_mission_ack(_m, master)
            mission_upload_active = False
            
            future = self.mission_upload_client.send_request(mission_upload_sess.mission_item_list)

            rclpy.spin_until_future_complete(self.mission_upload_client, future)

            response = future.result()

            if response.success:
                self.mission_upload_client.get_logger().info(f"Mission upload succeeded: {response.success}")
            else:
                self.mission_upload_client.get_logger().error(f"Mission upload failed: {response.success}")


    def handle_command_long(self, m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile) -> None:
        """
        Handles COMMAND_LONG sent from QGC.

        Examples:
        - request autopilot capabilities
        - arm/disarm
        - waypoint-related commands
        """
        global mvl_armed

        cmd = m.command

        # QGC is asking what this MAVLink endpoint supports
        if cmd == mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            if int(m.param1) == 1:
                capabilities = 0
                capabilities |= mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
                capabilities |= mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2
                capabilities |= mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE

                master.mav.autopilot_version_send(
                    capabilities=capabilities,
                    flight_sw_version=2,
                    middleware_sw_version=1,
                    os_sw_version=0,
                    board_version=1,
                    flight_custom_version=b"\x00" * 8,
                    middleware_custom_version=b"\x00" * 8,
                    os_custom_version=b"\x00" * 8,
                    vendor_id=10101,
                    product_id=20202,
                    uid=0
                )

        # QGC is asking to arm or disarm
        elif cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            mvl_armed = (int(m.param1) == 1)

            master.mav.command_ack_send(
                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
            )

        # Waypoint/nav command received
        # elif cmd == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
        #     print("WAYPOINT COMMAND RECEIVED")

        # else:
        #     print("OTHER CMD RECEIVED")

    def send_mission_request_int(self, m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile):
        """
        Requests mission items one by one from QGC.
        """
        global mission_upload_active
        global mission_upload_sess
        
        # If we haven't seen all the items yet, ask for the next
        print(f"REQUESTING {mission_upload_sess.current_mission_item}")
        master.mav.mission_request_int_send(
            target_system=self.comms_config.MVL_SYSID,
            target_component=self.comms_config.MVL_COMPID,
            seq=mission_upload_sess.current_mission_item
        )
        mission_upload_sess.is_waiting = True # Flag that mission upload is awaiting a response
        mission_upload_sess.t_last_transmit = millis()


    def send_mission_ack(self, _m: mavutil.mavlink.MAVLink_message, master: mavutil.mavfile):
        master.mav.mission_ack_send(
                target_system=self.comms_config.MVL_SYSID,
                target_component=self.comms_config.MVL_COMPID,
                type=0
            )


    def send_heartbeat(self, master: mavutil.mavfile) -> None:
        """
        Send heartbeat periodically so QGC knows this MAVLink component is alive.
        """
        base_mode = (
            mavutil.mavlink.MAV_MODE_MANUAL_ARMED
            if mvl_armed else mavutil.mavlink.MAV_MODE_MANUAL_DISARMED
        )

        # Add custom mode enabled flag
        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

        master.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_SUBMARINE,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            base_mode=base_mode,
            custom_mode=0xABBA,
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE
        )


    def send_sys_status_and_att(self, master: mavutil.mavfile) -> None:
        """
        Periodically send SYS_STATUS and ATTITUDE_QUATERNION.
        This helps QGC display telemetry from your system.
        """
        global sys_stat_count

        voltage_mv = sys_stat_count * 100 + 10000
        sys_stat_count = (sys_stat_count + 1) % 60

        master.mav.sys_status_send(
            onboard_control_sensors_present=0,
            onboard_control_sensors_enabled=0,
            onboard_control_sensors_health=0,
            load=0,
            voltage_battery=voltage_mv,
            current_battery=-1,
            battery_remaining=-1,
            drop_rate_comm=0,
            errors_comm=0,
            errors_count1=0,
            errors_count2=0,
            errors_count3=0,
            errors_count4=0
        )

        master.mav.attitude_quaternion_send(
            time_boot_ms=millis(),
            q1=1.0,
            q2=0.0,
            q3=0.0,
            q4=0.0,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0
        )


def main() -> None:

    rclpy.init()

    node = GCSInterface()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Entry point
if __name__ == "__main__":
    main()
