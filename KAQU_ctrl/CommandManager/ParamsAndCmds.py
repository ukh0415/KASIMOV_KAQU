#!/usr/bin/env python3

# 계산 및 제어에 관련한 변수들을 한번에 모아두는 곳. 
# 구현 안할 내용들은 일단 주석처리. 

import numpy as np
from enum import Enum
# 통신 및 센서 관련
USE_IMU = False
USE_OHTER_SENSORS = False

RATE = 60

# param - 지오메트리 관련
class BodyParam():
    # 대부분의 내용은 urdf에 있지만, 일단 한번에 모아보기 좋게 나눠봄. 
    def __init__(self):
        self.height = None
        self.centerOfMass = np.zeros([3])
        self.physical = self._physical_params()
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.ZMP_handler = np.zeros([4,3])
    class _physical_params():
        _length = 1
        _width = 1
        _min_height = 1
        _max_height = 1

class _LegParam:
    def __init__(self):
        self.pose = self.leg_pose()
        self.gait = self.gait_param()
    class gait_param:
        def __init__(self):
            self.cycle_time = None
            self.stance_time = 0.18
            self.swing_time = 0.24
            self.time_step = 0.02
            self.use_imu = USE_IMU

    class leg_pose:
        def __init__(self):
            self.default_stance = default_stance
            self
@property
def default_stance(self):
    # FR, FL, RR, RL
    return np.array([],[],[])

class Teleop():
    USE_JOY = True
    USE_KEYBOARD = False



init_pose

default_pose
    default_height


# 행동 관련 변수들
class BehaviorState(Enum):
    # START_UP = 10 # 첨에 자리에서 일어나는 등의 기능을 넣을 때.
    REST = 0
    TROT = 1
    # CRAWL = 2
    STAND = 3
    # OTHER_BEHAVIOR = (int)

# 현재 로봇의 상태
class DynamicState(object):
    def __init__(self, default_height):
        self.velocity = np.array([0.,0.])
        self.yaw_rate = 0.
        self.robot_height = -default_height

        self.foot_locations = np.zeros((3,4))

        self.body_local_position = np.array([0., 0., 0.])
        self.body_local_orientation = np.array([0., 0., 0.])

        self.imu_roll = 0.
        self.imu_pitch = 0.
        self.imu_yaw = 0.

        self.ticks = 0
        self.behavior_state = BehaviorState.REST


# 커멘드
class Command(object):
    def __init__(self, default_height):
        self.cmd_vel = np.array([0., 0.])
        self.cmd_yaw_rate = 0.
        self.robot_height = -default_height

        self.trot_event = False
        self.crawl_event = False
        self.rest_event = False
        self.stand_event = False
        # self.other_behavior_event = False








    

