import rclpy
import numpy as np
from InverseKinematics import robot_IK
from PIDController import PID_controller
from RobotUtilities.Transformations import rotxyz

# 접지 상태일 때의 위치 제어

class RestController(object):
    def __init__(self, default_stance):
        self.def_stance = default_stance # 초기 다리 위치

        # TODO : 게인값 조율
        self.pid_controller = PID_controller(0., 0., 0.)    # stance 상태이므로 모두 0
        self.use_imu = False                                # IMU 센서 사용 여부 = X
        self.use_button = True                              # IMU 센서 사용 여부 전환 버튼 기능 = O
        self.pid_controller.reset()                         # PID 초기화
        
        # msg를 기반으로 로봇의 상태(state)와 명령(command) 업데이트
    def updateStateCommand(self, msg, state, command):
        # local body position / orientation
        state.body_local_position[0] = msg.axes[7]*0.04
        state.body_local_position[1] = msg.axes[6]*0.03
        state.body_local_position[2] = msg.axes[1]*0.03
        # 조이스틱 값으로 몸체의 위치 변경

        state.body_local_orientation[0] = msg.axes[0]*0.4
        state.body_local_orientation[1] = msg.axes[4]*0.5
        state.body_local_orientation[2] = msg.axes[3]*0.4
        # 조이스틱 값으로 몸체의 방향(r-p-y) 변경

        if self.use_button:
            if msg.buttons[7]: # 7번 버튼으로 IMU 센서 사용 여부 전환
                self.use_imu = not self.use_imu
                self.use_button = False
                print(f"RESTController - Use rp compensation : {self.use_imu}")
        if not self.use_button:
            if not (msg.buttons[7]):
                self.use_button = True
    @property
    def default_stance(self):
        return self.def_stance
    
        # 상태(state)와 명령(command)을 기반으로 발 위치 계산
    def step(self, state, command):
        temp = self.default_stance # temp에 초기 다리 위치 대입
        temp[2] = [command.robot_height]*4

        # rp compensation
        # 나중에 이 부분 수정하면 됨
        if self.use_imu: # IMU 센서 활성화 된 경우
            compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
            roll_compensation = -compensation[0]
            pitch_compensation = -compensation[1]
            # PID 제어로 roll, pitch 값 보정

            rot = rotxyz(roll_compensation, pitch_compensation, 0) # 보정 값으로 rot 행렬 생성
            temp = np.matmul(rot, temp) # 보정된 위치 계산
        return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command) # 최종 발 위치 계산
        return state.foot_locations
                


