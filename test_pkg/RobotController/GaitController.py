#!/usr/bin/env python3
#Author: mike4192 https://github.com/mike4192/spotMicro
#Modified by: lnotspotl

# 주어진 contact_phases 에서의 전반적인 변수들 제어

import numpy as np
# 현재 어떤  phase에 있고 얼마나 지났는지, 어떤 다리가 접지인지 스윙인지
class GaitController(object): # class 생성
    def __init__(self, stance_time, swing_time, time_step, contact_phases, default_stance):
        self.stance_time = stance_time          # 접지 시간
        self.swing_time = swing_time            # 스윙 시간
        self.time_step = time_step              # 한 tick의 시간 단위
        self.contact_phases = contact_phases    # 각 단계에서 어떤 다리가 접지 중인지 나타내는 행렬(4x4)
        self.def_stance = default_stance        # 초기 다리 위치

    # @property > 반환만 함

    @property  # 초기 다리 위치를 불러옴
    def default_stance(self): 
        return self.def_stance

    @property  # 접지 틱
    def stance_ticks(self):
        return int(self.stance_time / self.time_step)

    @property  # 스윙 틱
    def swing_ticks(self): 
        return int(self.swing_time / self.time_step)

    @property  # phase 틱
    def phase_ticks(self):
        temp = []
        for i in range(len(self.contact_phases[0])):
            if 0 in self.contact_phases[:,i]:
                temp.append(self.swing_ticks)
            else:
                temp.append(self.stance_ticks)
        return temp
        # contact_phase에 따라 각 단계가 몇 틱 동안 지속되는지 계산
        # i번째 열의 중에 0이 하나라도 존재하면 스윙 틱을, 모두 1이라면 없으면 접지 틱을 대입하여 1x4 행렬 생성

    @property
    def phase_length(self):
        return sum(self.phase_ticks)
        # phase_ticks의 4가지 성분 총 합 / 총 틱

		# 현재 몇 번째 phase에 있는지 계산
    def phase_index(self, ticks):  
        """ Calculate, which part of the gait cycle the robot should be in """
        phase_time = ticks % self.phase_length # 틱에서 주기를 나눈 나머지
        phase_sum = 0
        phase_ticks = self.phase_ticks
        for i in range(len(self.contact_phases[0])):
            phase_sum += phase_ticks[i] # 각 phase 별 틱
            if phase_time < phase_sum:                  
                return i
        assert False
        # 몇 번째 phase에 나머지(phase_time)가 존재하는지 계산

		# 현재 phase에서 몇 틱 지났는지 계산
    def subphase_ticks(self, ticks):
        """ Calculate the number of ticks (timesteps)
            since the begining of the current phase """
        phase_time = ticks % self.phase_length 
        phase_sum = 0
        phase_ticks = self.phase_ticks
        for i in range(len(self.contact_phases[0])):
            phase_sum += phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - phase_sum + phase_ticks[i]
                return subphase_ticks
        assert False
        # phase_index와 유사

    	# 특정 틱에서의 접지, 스윙 여부 판단
    def contacts(self, ticks):
        """ Calculate which feet should be in contact """
        return self.contact_phases[:, self.phase_index(ticks)]