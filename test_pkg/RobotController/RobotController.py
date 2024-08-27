#!/usr/bin/evn python3
# 
import rclpy
from rclpy.node import Node
from CommandManager import ParamsAndCmds
from CommandManager.ParamsAndCmds import BodyParam, LegParam, Command, DynamicState, BehaviorState
from RobotUtilities.Transformations import euler_from_quaternion



# 커멘드 가져오기

# 각 컨트롤러에서 클래스 오브젝트 가져오기
from RestController import RestController
from StandController import StandController
from TrotGaitController import TrotGaitController
# IK 가져오기


# gait 별 컨트롤러 총괄

class Robot(object):
    def __init__(self, body, legs, imu):
        self.body = body
        self.legs = legs

        self.delta_x = body._physical_params._length*0.5
        self.delta_y = body._physical_params._width*0.5 + legs._physical_params.l1
        self.x_shift_front = 0.006
        self.x_shift_back = -0.03
        self.default_height = 0.15
        
        # 컨트롤러 호출을 위한 기본 발 위치 설정. 요런식으로 불러와도 되는지를 모르겠네
        self.default_stance = ParamsAndCmds.default_stance()
        
        # 각 컨트롤러 오브젝트 정의
        self.trotGaitController = TrotGaitController(self.default_stance,
            stance_time = legs.gait_param.stance_time,
            swing_time = legs.gait_param.swing_time,
            time_step = legs.gait_param.time_step) # 요부분 legs가 아니라 LegParam으로 바꿔야 할 수도 있음. 
        self.restController = RestController(self.default_stance)
        self.standController = StandController(self.default_stance)
        
        self.currentController = self.restController
        self.state = DynamicState(self.default_height)
        self.state.foot_locations = self.default_stance
        self.command = Command(self.default_height)
        
        # 여러 함수
    def change_controller(self):
        if self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
            self.command.trot_event = False

        # elif self.command.crawl_event:
        #     if self.state.behavior_state == BehaviorState.REST:
        #         self.state.behavior_state = BehaviorState.CRAWL
        #         self.currentController = self.crawlGaitController
        #         self.currentController.first_cycle = True
        #         self.state.ticks = 0
        #     self.command.crawl_event = False

        elif self.command.stand_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.STAND
                self.currentController = self.standController
            self.command.stand_event = False

        elif self.command.rest_event:
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False
    
    # 조이스틱 메시지 콜백
    def joystick_command(self,msg):
        if msg.buttons[0]: #rest
            self.command.trot_event = False
            self.command.crawl_event = False
            self.command.stand_event = False
            self.command.rest_event = True

        elif msg.buttons[1]: #trot
            self.command.trot_event = True
            self.command.crawl_event = False
            self.command.stand_event = False
            self.command.rest_event = False

        elif msg.buttons[2]: #crawl
            self.command.trot_event = False
            self.command.crawl_event = True
            self.command.stand_event = False
            self.command.rest_event = False

        elif msg.buttons[3]: #stand
            self.command.trot_event = False
            self.command.crawl_event = False
            self.command.stand_event = True
            self.command.rest_event = False

        self.currentController.updateStateCommand(msg, self.state, self.command)

    #imu 메시지 콜백
    def imu_orientation(self, msg):
        q = msg.orientation
        rpy_angles = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.state.imu_roll = rpy_angles[0]
        self.state.imu_pitch = rpy_angles[1]
    
    def run(self):
        return self.currentController.run(self.state, self.command)
    @property
    def default_stance(self):
        return self.default_stance