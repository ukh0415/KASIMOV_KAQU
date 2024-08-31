import rclpy
import numpy as np
from InverseKinematics import robot_IK
from PIDController import PID_controller

class TrotGaitController(object):
    def __init__(self, default_stance):
        self.default_stance = default_stance

        # TODO : 게인값 조율
        self.pid_controller = PID_controller(0., 0., 0.)
        self.use_imu = False
        self.use_button = True
        self.pid_controller.reset()
        



