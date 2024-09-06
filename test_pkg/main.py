#!/usr/bin/env python3

# 일단 이 노드는 다른 노드를 깨우고, 전부 다 중개하는 노드. 임포트만 제대로 되면 잘 실행될 것. 
# 조이스틱으로부터 메시지 받음

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, Imu
import sys
sys.path.append('/home/apka/ros2_ws/src/test_pkg/test_pkg/RobotController')
sys.path.append('/home/apka/ros2_ws/src/test_pkg/test_pkg/InverseKinematics')
sys.path.append('/home/apka/ros2_ws/src/test_pkg/test_pkg/CommandManager')
sys.path.append('/home/apka/ros2_ws/src/test_pkg/test_pkg/JoyNode')
from test_pkg.RobotController import RobotController
from InverseKinematics import robot_IK
from CommandManager import ParamsAndCmds

# from RobotController import RobotController
# from RobotController.RobotController import Robot
from InverseKinematics import robot_IK

from CommandManager.CmdManager import CmdManager_ROS2
from std_msgs.msg import Float64

# 센서 및 통신 관련 설정




# 변수 가져오기
body = ParamsAndCmds.BodyParam()
legs = ParamsAndCmds.LegParam()
default_stance = ParamsAndCmds.LegParam.leg_pose.def_stance
init_pose = ParamsAndCmds.LegParam.leg_pose.initial_pose
default_height = ParamsAndCmds.BodyParam.default_height
USE_IMU = ParamsAndCmds.Interface.USE_IMU
RATE = ParamsAndCmds.Interface.RATE
cmd = ParamsAndCmds.Command(default_height=default_height)

body_area ={body.physical._length, body.physical._width}
leg_length = {legs.physical.l1, legs.physical.l2, legs.physical.l3, legs.physical.l4}

# 클래스 선언
KAQU_robot =RobotController.Robot(body_area, leg_length, USE_IMU)
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
    except KeyboardInterrupt:
        KAQU_cmd_manager.node.destroy_node()
        rclpy.shutdown()

if __name__ =='__main__':
    main()