import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Float64
from RobotController import RobotController
from InverseKinematics import robot_IK
import ParamsAndCmds

# 서브스크라이버 콜백은 편의상 RobotController에 정의되어 있고, 
# 퍼블리셔는 여기에 정의되어 있음. 

interface = ParamsAndCmds.Interface
USE_IMU = not interface.USE_IMU
RATE = interface.RATE
body = ParamsAndCmds.BodyParam
legs = ParamsAndCmds.LegParam
KAQU_robot = RobotController.Robot(body, legs, USE_IMU)

joint_topics = ["/KAQU_ctrl/FR1_joint/command",
                "/KAQU_ctrl/FR2_joint/command",
                "/KAQU_ctrl/FR3_joint/command",
                "/KAQU_ctrl/FL1_joint/command",
                "/KAQU_ctrl/FL2_joint/command",
                "/KAQU_ctrl/FL3_joint/command",
                "/KAQU_ctrl/RR1_joint/command",
                "/KAQU_ctrl/RR2_joint/command",
                "/KAQU_ctrl/RR3_joint/command",
                "/KAQU_ctrl/RL1_joint/command",
                "/KAQU_ctrl/RL2_joint/command",
                "/KAQU_ctrl/RL3_joint/command"]

class CmdManager_ROS2():
    def __init__(self, set_msgs, send_msgs, node_name = 'cmd_manager_node'):
        super(CmdManager_ROS2, self).__init__()

        
        #ROS params
        self.node = None
        self.node_name = node_name
        # ---joy sub
        self.sub1 = None
        self.sub1_topic = "KAQU_joy/joy_ramped"
        self.sub1_msgType = Joy
        self.sub1_cb = KAQU_robot.joystick_command
        self.sub1_queueSize = 30
        # ---imu sub
        self.sub2 = None
        self.sub2_topic = "KAQU_imu/base_link_orientation"
        self.sub2_msgType = Imu
        self.sub2_cb = KAQU_robot.imu_orientation
        self.sub2_queueSize = 10
        # --- pub
        self.pub1 = None
        self.pub1_topic = joint_topics
        self.pub1_msgType = 하준이가 보내주는거
        self.pub1_timer_period = 0.01
        self.pub1_timers = []
        self.pub1_queue = 10
        self.joint_publishers1 = []
        self.pub1_cb = self._joint_pub_cb
    def _createNode(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.node_name)
    
    def create_sub1(self):
        self.sub1 = self.node.create_subscription(
                        self.sub1_msgType,
                        self.sub1_topic,
                        self.sub1_cb,
                        self.sub1_queueSize)
    def create_sub2(self):
        self.sub1 = self.node.create_subscription(
                        self.sub2_msgType,
                        self.sub2_topic,
                        self.sub2_cb,
                        self.sub2_queueSize)
    def create_pub1(self):
        for i in range(len(joint_topics)):
            self.joint_publishers1[i] = self.node.create_publisher(
                                            self.pub1_msgType,
                                            self.pub1_topic,
                                            self.pub1_queue)
            


    def _joint_pub_cb(self):







