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

class RobotController(Node):
    def __init__(self):
        super.__init__('ctrl_gz_node')

        # 여러 설정

        # 퍼블리셔





def start(self):
    rclpy.shutdown()


def main(args=None):
    print("Starting Robot")

    # 노드 켜는 부분

    
    rclpy.shutdown()




if __name__ =='__main__':
    main()