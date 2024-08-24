#!/usr/bin/env python3

# 일단 이 노드는 다른 노드를 깨우고, 전부 다 중개하는 노드. 임포트만 제대로 되면 잘 실행될 것. 
# 조이스틱으로부터 메시지 받음

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from CommandManager import ParamsAndCmds
from std_msgs.msg import Float64

# 센서 및 통신 관련 설정


# 클래스 선언
robot = RobotController()

# 변수 가져오기
default_stance = ParamsAndCmds.default_stance



def start(self):
    rclpy.shutdown()


def main(args=None):
    print("Starting Robot")

    # 노드 켜는 부분

    # init 포즈 설정

    try:
        #각 노드.run

    
    rclpy.shutdown()




if __name__ =='__main__':
    main()