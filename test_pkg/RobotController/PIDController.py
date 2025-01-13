#!/usr/bin/env python3

# 균형 제어 / 오차 관련

import rclpy
from rclpy.clock import Clock, ClockType
import numpy as np

import rclpy.time

class PID_controller(object):
    def __init__(self, kp, ki, kd):
        self.kp = kp    # proportion 비례
        self.ki = ki    # integral 적분
        self.kd = kd    # differentiatial 미분

        self.desired_roll_pitch = np.array([0.0, 0.0])  # 2차원 (r-p)
        self.desired_rpy = np.array([0.0, 0.0, 0.0])

        # TODO : Tune max_I
        self.max_I = 0.2
        self.last_error = np.array([0.0,0.0]) # 이전에 대한 오차 초기화

        self.clock = Clock(clock_type=ClockType.ROS_TIME) # 현재 시간
        
        # 변수 초기화
    def reset(self):
        self.last_time = self.clock.now()
        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])
        self.last_error = np.array([0.0,0.0])
    
        # 2차원 (r-p)에 대한 PID 제어값
    def run(self, roll, pitch):
        error = self.desired_roll_pitch - np.array([roll, pitch]) # 현재 error

        t_now = self.clock.now()
        step, step_millis = (t_now - self.last_time).seconds_nanoseconds()
        # step : 초 단위 차이
        # step_millis : 나노 초 단위 차이

        step = step+step_millis

        self.I_term = self.I_term + error*step  # 적분항
        
        for i in range(2):
            if(self.I_term[i] < -self.max_I):
                self.I_term[i] = -self.max_I
            elif(self.I_term[i] > self.max_I):
                self.I_term[i] = self.max_I
        
        self.D_term = (error - self.last_error) / step  # 미분항

        self.last_time = t_now
        self.last_error = error

        P_ret = self.kp * error
        I_ret = self.ki * error
        D_ret = self.kd * error

        return P_ret+I_ret+D_ret
    
    # def run_rpy(self, roll, pitch, yaw) > 3차원(r-p-y)

    def desired_RP_angles(des_roll, des_pitch):
        self.desired_roll_pitch = np.array([des_roll, des_pitch])