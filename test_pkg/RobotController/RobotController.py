#!/usr/bin/evn python3
# 
import rclpy
from rclpy.node import 

from CommandManager.ParamsAndCmds import BodyParam, LegParam, 



# 커멘드 가져오기

# 각 컨트롤러에서 클래스 오브젝트 가져오기
# IK 가져오기


# gait 별 컨트롤러 총괄

class RobotController(Node):
    def __init__(self, body, legs, imu):
        super.__init__("robot_controller_node")
        self.body = body
        self.legs = legs

        self.delta_x = body._physical_params._length*0.5
        self.delta_y = body._physical_params._width*0.5 + legs._physical_params.l1
        self.x_shift_front = 0.006
        self.x_shift_back = -0.03
        self.default_height = 0.15

        # 각 컨트롤러 오브젝트 정의



        self.currentController = self.restController
        
        # 여러 함수
        def change_controller(self):
            if self.

        # 리턴할 내용 : 발의 xyz 위치

        # start 해서 스핀하는 내용 들어가야 함. 