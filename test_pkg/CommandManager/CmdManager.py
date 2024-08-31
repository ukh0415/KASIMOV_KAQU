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

body_area ={body.physical._length, body.physical._width}
leg_length = {legs.physical.l1, legs.physical.l2, legs.physical.l3, legs.physical.l4}

KAQU_robot = RobotController.Robot(body_area, leg_length, USE_IMU)
KAQU_ik = robot_IK.InverseKinematics(body_area, leg_length)

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
        self.sub1_topic = "cmd_vel"
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
        self.pub1_timer = None
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
            self.pub1_timer = self.node.create_timer(self.pub1_timer_period, self.pub1_cb)
            


    def _joint_pub_cb(self):
        leg_positions = KAQU_robot.run()
        KAQU_robot.change_controller()

        dx = KAQU_robot.state.body_local_position[0]
        dy = KAQU_robot.state.body_local_position[1]
        dz = KAQU_robot.state.body_local_position[2]

        roll = KAQU_robot.state.body_local_orientation[0]
        pitch = KAQU_robot.state.body_local_orientation[1]
        yaw = KAQU_robot.state.body_local_orientation[2]

        try:
            joint_angles = KAQU_ik.inverse_kinematics(leg_positions, dx, dy, dz, roll, pitch, yaw)

            for i in range(len(joint_angles)):
                self.joint_publishers1[i].publish(joint_angles[i])
        except:
            pass

    def start(self):
        self._createNode()
        self.create_sub1()
        self.create_sub2()
        self.create_pub1()
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()
        self.stop = False
        
    def stop(self):
        self.stop = True










