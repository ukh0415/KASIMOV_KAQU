import rclpy
import numpy as np
from InverseKinematics import robot_IK
from PIDController import PID_controller
from RobotUtilities.Transformations import rotxyz

# 접지 상태일 때의 위치 제어

class RestController(object): # object 상속 : python2 에서 구형 class를 신형 class로 바꾸기 위한 작업
    def __init__(self, default_stance): # 생성자 / self : 클래스 내의 모든 메서드(클래스 내의 함수)는 self를 첫 번째 매개변수로 가져야함
        self.def_stance = default_stance # 초기 다리 위치 / def_stance : 변수

        # TODO : 게인값 조율 (실험적으로)
        self.pid_controller = PID_controller(0., 0., 0.)    # stance 상태이므로 모두 0
        self.use_imu = False                                # IMU 센서 사용 여부 = X
        self.use_button = True                              # IMU 센서 사용 여부 전환 버튼 기능 = O
        self.pid_controller.reset()                         # PID 초기화

    #def __init__(self, default_stance, pid_gains=(0., 0., 0.)):
    #   self.def_stance = default_stance
    #   self.pid_controller = PID_controller(*pid_gains)
    # GPT의 추천 : 게인값을 외부에서 입력 받거나 설정 파일로 관리 하면 유연성 높아짐

        
        # msg를 기반으로 로봇의 상태(state)와 명령(command) 업데이트 / msg > 조이스틱 입력
        # state는 ParamsAndCmds.py의 DynamicState 사용 > state = DynamicState()
    def updateStateCommand(self, msg, state, command):
        # local body position / orientation
        # msg.axes : 조이스틱 축 값 / *0.04 : 실험적 비율로 스케일링
        state.body_local_position[0] = msg.axes[7]*0.04 # X 방향 / 0.04배율
        state.body_local_position[1] = msg.axes[6]*0.03 # Y 방향 / 0.03배율
        state.body_local_position[2] = msg.axes[1]*0.03 # Z 방향 / 0.03배율
        # 조이스틱 값으로 몸체의 위치 변경

        state.body_local_orientation[0] = msg.axes[0]*0.4 # roll  방향 / 0.4배율
        state.body_local_orientation[1] = msg.axes[4]*0.5 # pitch 방향 / 0.5배율
        state.body_local_orientation[2] = msg.axes[3]*0.4 # yaw   방향 / 0.4배율
        # 조이스틱 값으로 몸체의 방향(r-p-y) 변경

        # IMU 센서 키고 끄기
        if self.use_button: # 버튼이 눌렸을 때
            if msg.buttons[7]:                       # 7번 버튼 입력하면
                self.use_imu = not self.use_imu      # IMU 센서 사용여부 전환
                self.use_button = False              # 버튼 중복 입력으로 인한 오작동 방지 / 한 번 눌렸을때만 작동하게끔 / 잠시 비활성화 / True > False
                print(f"RESTController - Use rp compensation : {self.use_imu}") # IMU 센서 기반의 Roll-Pitch 보정(rp compensation) 사용 여부 출력
                # f-string : 문자열 안에 변수를 간단하게 삽입할 수 있는 방식 / self.use_imu 의 값이 문자열로 출력 (ex: False)
        if not self.use_button: # 버튼이 떼어졌을 때
            if not (msg.buttons[7]):                 # 7번 버튼 떼어지면
                self.use_button = True               # 다시 버튼 입력 활성화 / False > True 

     # POSITION_SCALING = [0.04, 0.03, 0.03]
     # ORIENTATION_SCALING = [0.4, 0.5, 0.4]
     # GPT의 추천 : 스케일링 값을 상수로 정의하거나 별도 파일로 관리하면 유지보수에 좋음


    @property # 메서드를 속성값 처럼 사용
    def default_stance(self):
        return self.def_stance
    
        # 상태(state)와 명령(command)을 기반으로 발 위치(높이) 조절
    def step(self, state, command):
        temp = self.default_stance # temp에 초기 다리 위치 대입
                                   # def_stance = np.array([[0, 0, 0, 0],         X
                                   #                        [0, 0, 0, 0],         Y
                                   #                        [0, 0, 0, 0]])        Z
                                   #                   다리1, 다리2, 다리3, 다리4
        temp[2] = [command.robot_height]*4 # 좌변 = 3번째 행 / 우변 = command.robot_height 값이 4번 출력(ex: [1.5, 1.5, 1.5, 1.5])
        # 네 다리의 높이를 command를 통해 지정

        # rp compensation
        # 나중에 이 부분 수정하면 됨
        if self.use_imu: # IMU 센서 활성화 된 경우
            compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch) # run : PIDController의 메서드 / run(self, roll, pitch)
            roll_compensation = -compensation[0] # roll
            pitch_compensation = -compensation[1] # pitch
            # PID 제어로 roll, pitch 값 보정

            rot = rotxyz(roll_compensation, pitch_compensation, 0) # 보정 값으로 rot 행렬 생성
            temp = np.matmul(rot, temp) # 보정된 위치 계산
        return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command) # 최종 발 위치
        return state.foot_locations
                


