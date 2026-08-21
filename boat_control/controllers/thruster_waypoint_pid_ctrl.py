import time
import math
import numpy as np
from boat_control.data.ctrl_coeffs import CtrlCoeffs
from boat_control.helpers.math_helpers import *

class ThrusterPIDControl:
    def __init__(self, ctrl_coeffs:CtrlCoeffs):
        self.dist_error_buff = np.zeros(10) # Index 0 is most recent
        self.hdg_error_buff = np.zeros(10) # Index 0 is most recent
        self.coeffs = ctrl_coeffs
        self.last_ctrl_time = None

    def controller(self, lat, lon, goal_lat, goal_lon, hdg) -> Tuple[int, int]:
        # Distance controller
        dist_err = distance_between_coordinates(goal_lat, goal_lon, lat, lon)

        int_dist_err = clamp(np.sum(self.dist_err_buff), -50, 50) # Clamp to prevent integral windup
        now = time.monotonic()
        d_dist_err_dt = 0.0

        if self.last_ctrl_time is not None:
            dt = now - self.last_ctrl_time
            d_dist_err_dt = (dist_err - self.dist_err_buff[0]) / dt

        dist_ctrl = (self.coeffs.kp_d * dist_err + 
            self.coeffs.ki_d * int_dist_err +
            self.coeffs.kd_d * d_dist_err_dt)
        
        dist_ctrl = clamp(dist_ctrl, -1, 1) # Clamping

        self.dist_err_buff = np.roll(self.dist_err_buff, 1) # Shift all values right
        self.dist_err_buff[0] = dist_err
        
        # Heading controller
        goal_hdg = angle_between_coordinates(lat, lon, goal_lat, goal_lon)
        hdg_error = wrap_angle_deg(goal_hdg - hdg)
        int_hdg_err = clamp(np.sum(self.hdg_err_buff), -50, 50)
        d_hdg_err_dt = 0.0
        if self.last_ctrl_time is not None:
            dt = now - self.last_ctrl_time
            hdg_error_delta = wrap_angle_deg(hdg_err - self.hdg_err_buff[0])
            d_hdg_err_dt = hdg_error_delta / dt

        hdg_ctrl = (self.coeffs.kp_h * hdg_err + 
            self.coeffs.ki_h * int_hdg_err + 
            self.coeffs.kd_h * d_hdg_err_dt)

        hdg_ctrl = clamp(hdg_ctrl, -1, 1) # Clamping

        self.hdg_err_buff = np.roll(self.hdg_err_buff, 1)
        self.hdg_err_buff[0] = hdg_err

        self.last_ctrl_time = now
        
        # Mixing
        left_power = dist_ctrl - hdg_ctrl
        right_power = dist_ctrl + hdg_ctrl

        norm_factor = max(1, max(abs(left_power), abs(right_power)))
        left_power = left_power / norm_factor
        right_power = right_power / norm_factor

        return left_power, right_power




    
