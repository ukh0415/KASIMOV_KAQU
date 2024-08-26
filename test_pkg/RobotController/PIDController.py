#!/usr/bin/env python3

import rclpy
import numpy as np

class PID_controller(object):
    def __init__(self, kp. ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self):
        self.last_time = 여기에 타임 나우 해줘야 함. 
        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])
        self.last_error = np.array([0.0,0.0])