#!/usr/bin/env python
#-*-coding: utf-8-*-
import serial
import time
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np

from time import sleep

ser = serial.Serial('/dev/ttyUSB1',115200)

# Parameters
k = 0.1  # look forward gain
Lfc = 4.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.05  # [s] time tick
WB = 0.9  # [m] wheel base of vehicle


show_animation = True

# 차량 상태
class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

# 조향각 계산 함수
def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
    # print(delta)
    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def func(x):
	pass


def clc_find(num):
    # make normal value to u_int8 value
    while num > 255:
        num -= 256

    # make int to 8-digit String list
    digit_num = list('{0:08b}'.format(num))

    # Change Value
    for i in range(8):
        if digit_num[i] == '0':
            digit_num[i] = '1'
        else:
            digit_num[i] = '0'

    # make list to string
    result = ''.join(digit_num)
    # return 8-digit string to int value
    return int(result,2)


def serWrite(v, d):
	i=0
	data=[]
    # 제어패킷 생성
	writeBuffer(data, v, d)
    print(v)
    print(d)
    # 제어패킷 전송
	#print(data)
	ser.write(data)
	cv2.waitKey(25)

def writeBuffer(data, speed, steering):
    # 기어 기본값 0: 전진, 1:후진
	direction = 0
    # 속도
	speed_Lo = speed & 0xFF;
	speed_Hi = speed >> 8;
    # 조향각
	steering_Lo = steering & 0xFF;
	steering_Hi = steering >> 8;
    # CLC 계산
	sum_1  = direction+speed_Lo+speed_Hi+steering_Lo+steering_Hi +220 +5+13+10  ##Fixed Break values (To find the CLC)
	clc = clc_find(sum_1)
    # packet 작성
	data.append(0x53)
	data.append(0x54)
	data.append(0x58)
	data.append(direction)  #0x00 forward #0x01 backward
	data.append(speed_Lo)  ##speed_Lo
	data.append(speed_Hi)  ##speed_Hi
	data.append(steering_Lo)  ##steer_Lo
	data.append(steering_Hi)  ##steer_Hi
	data.append(0xDC)  #Brake_L
	data.append(0x05)  #Brake_H
	data.append(0x00)  #Encode #default 0
	data.append(0x0D)
	data.append(0x0A)
	data.append(clc)  #CLC


def main():
    #  target course
    # cx = np.arange(0, 50, 0.5)
    # cy = [math.sin(ix / 3.0) * ix / 2.0 for ix in cx]

    # 경로 정보 입력
    cx = [0, 5, 10, 30, 31]
    cy = [0, 10, -15, 0, 0]
    # 목표 속도 설정
    target_speed = 10.0 / 3.6  # [m/s]
    T = 100.0  # max simulation time

    # 시작 지점 설정
    state = State(x=-0.0, y=-6.0, yaw=90.0, v=0.0)
    
    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    # 현재 지점 기준 way point를 지나쳤는지 검사
    while T >= time and lastIndex > target_ind:
        # Calc control input
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle

        time += dt
        states.append(time, state)
        ## -------------------- 시리얼 통신 -------------------- ##
	    print("di" , di)
        # 제어패킷 전송
        # state.v * 3.6 = km/h 변환
        # * 10 값은 제어패킷 속도값(0~800)에 맞추기위한 임의의 상수 값, 추후 변환표를 보고 수정필요
        # di 단위 확인필요
        # 1500 -(di*700) 값은 제어패킷 조향값(1200~1500~1900)에 맞추기 위한 임의의 상소, 추후 변환표를 보고 수정필요
        serWrite(int(state.v * 3.6 * 10), 1500-int(di * 700))
        # 패킷 간 딜레이 설정
        sleep(0.05)
        ## -------------------------------#-------------------- ##
        # 움직이는 과정을 그래프 상에 표현, 실제 주행 명령 패킷 전송시 주석처리
        '''
        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.00001)
        '''
    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
