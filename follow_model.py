# 车的各种模型
import time
import random
import numpy as np
import carla
from globe import GlobeVar
import sympy as sp
import math
from car_control import Car_control


class Follow_model(object):
    '''跟驰模型类'''
    def __init__(self, car):
        self.car = car
        self.speed = car.get_speed(car.velocity)
        self.length = car.get_length()
        self.acc = self.car.get_accelerometer()
        self.control = Car_control(car)
        self.iv = car.iv

    def is_in_follow_range(self, vehicle):
        '''判断是否在跟驰模型范围'''
        if vehicle is None:
            return False
        waypoint = self.car.get_waypoint(vehicle)
        if self.car.road_id == waypoint.road_id and self.car.lane_id == waypoint.lane_id:
            if self.car.get_waypoint_distance(vehicle) < GlobeVar.FOLLOW_RANGE:
                return True
            else:
                return False
        else:
            return False

    def follow_model_two(self, car, distance):
        '''传统车跟驰模型'''
        '''距离为主函数通过waypoint计算'''
        car_length = car.get_length()
        vb = self.speed
        # 后车速度
        vf = car.get_speed()
        # 前车速度
        d = distance
        # 下面是公式转化部分
        a = -1.2
        v_p = 6.75 + 7.91 * np.tanh(0.13 * (d - self.length) - 1.57)
        s_d = max(
            1 + car_length + self.speed * 1.6 + pow(vb, 2) / a / 2 -
            pow(vf, 2) / a / 2, 1 + car_length)
        lam = 2 if d < 100 else 0
        acc = 0.41 * (v_p - vb) + lam * (1 - s_d / d)
        self.control.acc_control(acc)
        return acc

    def follow_model_one(self, car, car_acc, distance, T, alpha):
        '''获得跟驰模型加速度'''
        '''参数依次为：车，加速度，距离，敏感度，最优速度，反应速度'''
        vt = 6.75 + 7.91 * np.tanh(0.13 * (distance - self.length) - 1.57)
        vx = 7.91 * 0.13 * (1 - pow(np.tanh(0.13 *
                                            (distance) - car.length), 2), 1.57)
        # 三个表达式
        a1 = 2 / (2 * T + alpha**2 * T**2 * vx)
        a2 = 2 * alpha * vx / (2 + alpha**2 * T * vx)
        a3 = alpha**2 * T * vx / (2 + alpha**2 * T * vx)
        acc = a1 * (vt - self.car.speed) + a2 * (abs(self.car.speed -
                                                     car.speed)) + a3 * car.acc
        return acc

    def idm_follow_model(self, car, distance):
        '''智能车跟驰模型'''
        car_length = car.get_length()
        vb = self.speed
        # 后车速度
        vf = car.get_speed()
        # 前车速度
        d = distance
        # 下面是公式转化部分
        ss = 2 + vb * 1.2 + vb * (abs(vb - vf)) / (2 * pow(1.5 * 2, 0.5))
        k = 1
        if ss <= 0:
            k = 0
        acc = 1.5 * (1 - pow(vb / self.iv, 4) -
                     k * pow(ss / (d - car_length), 2))
        # self.control.acc_control(acc)
        return acc
