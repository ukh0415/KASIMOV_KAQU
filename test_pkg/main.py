#!/usr/bin/env python3

# 일단 이 노드는 다른 노드를 깨우고, 전부 다 중개하는 노드. 임포트만 제대로 되면 잘 실행될 것. 
# 조이스틱으로부터 메시지 받음

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from CommandManager import ParamsAndCmds
from CommandManager.CmdManager import CmdManager_ROS2
from std_msgs.msg import Float64

# 센서 및 통신 관련 설정




# 변수 가져오기
body = ParamsAndCmds.BodyParam()
legs = ParamsAndCmds.LegParam()
default_stance = ParamsAndCmds.default_stance()
init_pose = ParamsAndCmds.init_pose()
USE_IMU = ParamsAndCmds.Interface.USE_IMU
RATE = ParamsAndCmds.Interface.RATE
cmd = ParamsAndCmds.Command()

body_area ={body.physical._length, body.physical._width}
leg_length = {legs.physical.l1, legs.physical.l2, legs.physical.l3, legs.physical.l4}

# 클래스 선언
KAQU_robot = RobotController.Robot(body_area, leg_length, ParamsAndCmds.USE_IMU)
KAQU_cmd_manager = CmdManager_ROS2(set_msgs=cmd, send_msgs=[legs, body_area])

def start(self):
    rclpy.shutdown()


def main(args=None):
    print("Starting Robot")

    # 노드 켜는 부분
    # 로봇 컨트롤러
    # 가제보 노드
    # 조이스틱 노드

    # init 포즈 설정

    try:
        KAQU_cmd_manager.start()

        #각 노드.run
    except:
        pass

# 커멘드메니져, 조이 노드, 하드웨어 인터페이스 실행
    rclpy.shutdown()




if __name__ =='__main__':
    main()