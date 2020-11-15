# 车的各种模型
import time
import random
import numpy as np
import custom_car
import carla
from carla_enviroment import GlobeVar
import sympy as sp
import math


class Follow_model(object):
    '''跟驰模型类'''
    def __init__(self, car):
        self.car = car
        self.speed = car.get_speed(car.velocity)
        self.length = car.get_length()
        self.acc = car.get_accelerometer()

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
        '''跟驰模型'''
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
        return acc

    def follow_model_one(self, car, car_acc, distance, T, alpha):
        '''获得跟驰模型加速度'''
        '''参数依次为：车，加速度，距离，敏感度，最优速度，反应速度'''
        t = sp.Symbol('t', real=True)
        # 时间
        vt = 6.75 + 7.91 * np.tanh(0.13 * (distance - self.length) - 1.57)
        # 最优速度
        seg_alpha = sp.Function('seg_alpha')
        seg_beta = sp.Function('seg_beta')
        seg_gama = sp.Function('seg_gama')
        # 三个表达式
        seg_alpha = 2 / (2 * T + alpha**2 * T**2 * sp.diff(vt, t))
        seg_beta = 2 * alpha * sp.diff(vt,
                                       t) / (2 + alpha**2 * T * sp.diff(vt, t))
        seg_gama = alpha**2 * T * sp.diff(
            vt, t) / (2 + alpha**2 * T * sp.diff(vt, t))
        eq = seg_alpha * (
            vt - self.velocity) + seg_beta * self.velocity + seg_gama * car_acc
        result = sp.solve(sp.Eq(eq, 0), t)
        acc_max = vt.evalf(subs={t: result})
        return acc_max

