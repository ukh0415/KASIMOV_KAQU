import rclpy
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped
from InverseKinematics import robot_IK
from GaitController import GaitController
from PIDController import PID_controller
from CommandManager.ParamsAndCmds import LegParam

# trot 방식의 보행 제어
# trot 보행 : 두 개의 대각선 다리를 동시에 움직이는 방식

class TrotGaitController(GaitController): # GaitController class 상속
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
        self.use_imu = use_imu  # IMU 활성화
        self.use_button = True  # 버튼 활성화
        self.autoRest = True    # 명령 없을 시 자동 휴면 상태 여부 = O
        self.trotNeeded = True  # TrotGait 실행 여부 = O
        
        leg = LegParam()

        contact_phases = np.array([[1, 1, 1, 0],  # 0: Leg swing > 스윙
                                   [1, 0, 1, 1],  # 1: Moving stance forward > 접지
                                   [1, 0, 1, 1],  
                                   [1, 1, 1, 0]])
                                   #step1: 모든 다리 접지
                                   #step2: 2, 3다리 스윙
                                   #step3: 모든 다리 접지
                                   #step4: 1, 4다리 스윙

        z_error_constant = 0.02 * 4    # This constant determines how fast we move
                                       # toward the goal in the z direction
                                       # 높이 조절

        z_leg_lift = leg.gait.z_leg_lift # 다리 높이

        # 부모 class 상속
        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        self.max_x_vel = leg.gait.max_x_vel
        self.max_y_vel = leg.gait.max_y_vel
        self.max_yaw_rate = leg.gait.max_yaw_rate
        
        self.swingController = TrotSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
                                                   self.phase_length, z_leg_lift, self.default_stance)
                                                   # 스윙 제어

        self.stanceController = TrotStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
                                                     self.time_step, z_error_constant)
                                                    # 접지 제어

        # TODO : 게인값 조율
        self.pid_controller = PID_controller(0., 0., 0.) # PID 제어

        # 조이스틱 입력(msg)을 기반으로 이동 속도(cmd_vel), 회전 속도(cmd_yaw_rate) 
    def updateStateCommand(self, msg, state, command):
        command.cmd_vel[0] = msg.linear.x /1.5 * self.max_x_vel
        command.cme_vel[1] = msg.linear.y /1.5 * self.max_y_vel
        command.cmd_yaw_rate = msg.angular.z * self.max_yaw_rate

        # if self.use_button:
        #     if msg.buttons[7]:
        #         self.use_imu = not self.use_imu
        #         self.use_button = False
        #         rospy.loginfo(f"Trot Gait Controller - Use roll/pitch compensation: {self.use_imu}")

        #     elif msg.buttons[6]:
        #         self.autoRest = not self.autoRest
        #         if not self.autoRest:
        #             self.trotNeeded = True
        #         self.use_button = False
        #         rospy.loginfo(f"Trot Gait Controller - Use autorest: {self.autoRest}")
            
        # if not self.use_button:
        #     if not(msg.buttons[6] or msg.buttons[7]):
        #         self.use_button = True

        # 정지 > trotNeeded = False / 이동 명령 > trotNeeded = True
    def step(self, state, command):
        if self.autoRest: # 정지 상태
            if command.cmd_vel[0] == 0 and command.cmd_vel[1] == 0 and command.cmd_yaw_rate ==0:
                if state.ticks % (2*self.phase_length) ==0:
                    self.trotNeeded = False
            else:
                self.trotNeeded = True
        if self.trotNeeded: # 이동 명령
            contact_modes = self.contacts(state.ticks) # 접지, 스윙 상태 확인

            new_foot_locations = np.zeros((3,4)) # 명령
            for leg_index in range(4):
                contact_modes = contact_modes[leg_index]
                if contact_modes ==1:
                    new_location = self.stanceController.next_foot_location(leg_index, state, command)
                else:
                    swing_proportion = float(self.subphase_ticks(state.ticks))/float(self.swing_ticks) # 스윙 진행 비율 계산 (0에서 1 사이)

                    new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state, command)
                new_foot_locations[:, leg_index] = new_location
            
            # imu compensation > 보정
            if self.use_imu:
                compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
                roll_compensation = -compensation[0]
                pitch_compensation = -compensation[1]

                rot = rotxyz(roll_compensation, pitch_compensation, 0)
                new_foot_locations = np.matmul(rot, new_foot_locations)
            state.ticks +=1
            return new_foot_locations
        else:
            temp = self.default_stance
            temp[2] = [command.robot_height]*4
            return temp

        # 발 위치 계산, 업데이트
    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.robot_height = command.robot_height

        return state.foot_locations


     # 스윙 상태에서 다리 위치 제어   
class TrotSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length, z_leg_lift, default_stance):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance

    def raibert_touchdwon_location(self, leg_index, command): # touchdown / x-y 최종 발의 위치
        delta_pos_2d = command.cmd_vel * self.phase_length * self.time_step # 거리 = 속도 * 시간
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0]) # 2차원 (r-p) / yaw 값 0

        theta = self.stance_ticks * self.time_step * command.cmd_yaw_rate # 회전
        rotation = rotz(theta)

        return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos # 초기 위치에서 계산 된 값 반환
    
    def swing_height(self, swing_phase): # z축
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5)
        return swing_height_
    
    def next_foot_location(self, swing_prop, leg_index, state, command): # x-y-z
        assert swing_prop >= 0 and swing_prop <= 1 # prop 값이 0에서 1사이가 아닐 시 중단
        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command)

        time_left = self.time_step* self.swing_ticks * (1.0 - swing_prop)
        
        velocity = (touchdown_location - foot_location) / float(time_left) *\
             np.array([1, 1, 0])

        delta_foot_location = velocity * self.time_step
        z_vector = np.array([0, 0, swing_height_ + command.robot_height])
        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location
    
     # 접지 상태에서 다리의 위치 제어
class TrotStanceController(object):
    def __init__(self,phase_length, stance_ticks, swing_ticks, time_step, z_error_constant):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant


    def position_delta(self, leg_index, state, command):
        z = state.foot_locations[2, leg_index]

        step_dist_x = command.cmd_vel[0] *\
                      (float(self.phase_length)/self.swing_ticks)

        step_dist_y = command.cmd_vel[1] *\
                      (float(self.phase_length)/self.swing_ticks)

        velocity = np.array([-(step_dist_x/4)/(float(self.time_step)*self.stance_ticks), 
                             -(step_dist_y/4)/(float(self.time_step)*self.stance_ticks), 
                             1.0 / self.z_error_constant * (state.robot_height - z)])

        delta_pos = velocity * self.time_step
        delta_ori = rotz(-command.cmd_yaw_rate * self.time_step)
        return (delta_pos, delta_ori)
    
    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        (delta_pos, delta_ori) = self.position_delta(leg_index, state, command)
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
        return next_foot_location