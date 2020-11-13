# 车的各种模型
import time
import random
import numpy as np
import custom_car
import carla
from carla_enviroment import GlobeVar
import sympy as sp
import math


class Change_lane_model(object):
    def __init__(self, car):
        self.car = car
        self.speed = car.get_speed(car.velocity)
        self.length = car.get_length()
        self.acc = car.get_accelerometer()
        self.lb_vehicle = car.get_lb_vehicle()
        self.lf_vehicle = car.get_lf_vehicle()
        self.rb_vehicle = car.get_rb_vehicle()
        self.rf_vehicle = car.get_rf_vehicle()
        self.next_vehicle = car.get_next_vehicle()
        self.last_vehicle = car.get_last_vehicle()
        self.lane_change = None

    def road_change_model_left(self, vehicle):
        '''换道模型'''
        distance = self.car.get_waypoint_distance(vehicle, 0.5)
        vf = self.carcar.get_speed(vehicle.get_velocity())
        # 前车速度
        limit_speed = vehicle.get_speed_limit()
        lb_vehicle = self.lb_vehicle
        left_distance_b = self.car.get_waypoint_distance(lb_vehicle, 0.5)
        lb_vehicle_velocity = lb_vehicle.get_velocity()
        lb_vehicle_speed = self.car.get_speed(lb_vehicle_velocity)
        lf_vehicle = self.car.get_lf_car()
        left_distance_f = self.car.get_waypoint_distance(lf_vehicle, 0.5)
        lf_vehicle_velocity = lf_vehicle.get_velocity()
        lf_vehicle_speed = self.car.get_speed(lf_vehicle_velocity)
        '''下面是公式转换部分'''
        a = 5 * limit_speed
        if distance >= 5 * a:
            if random.random() < 0.1:
                return True
        else:
            if self.speed >= limit_speed:
                if distance < 2.4 * self.speed:
                    if left_distance_f > distance and left_distance_b < 1.2 * lb_vehicle_speed:
                        return True
            else:
                if lf_vehicle_speed > self.speed:
                    if lf_vehicle_speed > 1.5 * vf:
                        if left_distance_f > 1.2 * self.speed and left_distance_b > 1.2 * lb_vehicle_speed:
                            return True
                else:
                    if lf_vehicle_speed > 1.5 * vf:
                        if left_distance_f > 2.4 * self.speed and left_distance_b > 1.2 * lb_vehicle_speed:
                            return True

    def road_change_model_right(self, vehicle):
        '''换道模型'''
        distance = self.car.get_waypoint_distance(vehicle, 0.5)
        vf = self.carcar.get_speed(vehicle.get_velocity())
        # 前车速度
        limit_speed = vehicle.get_speed_limit()
        rb_vehicle = self.rb_vehicle
        right_distance_b = self.car.get_waypoint_distance(rb_vehicle, 0.5)
        rb_vehicle_velocity = rb_vehicle.get_velocity()
        rb_vehicle_speed = self.car.get_speed(rb_vehicle_velocity)
        rf_vehicle = self.car.get_rf_car()
        right_distance_f = self.car.get_waypoint_distance(rf_vehicle, 0.5)
        rf_vehicle_velocity = rf_vehicle.get_velocity()
        rf_vehicle_speed = self.car.get_speed(rf_vehicle_velocity)
        '''下面是公式转换部分'''
        a = 5 * limit_speed
        if distance >= 5 * a:
            if random.random() < 0.1:
                return True
        else:
            if self.speed >= limit_speed:
                if distance < 2.4 * self.speed:
                    if right_distance_f > distance and right_distance_b < 1.2 * rb_vehicle_speed:
                        return True
            else:
                if rf_vehicle_speed > self.speed:
                    if rf_vehicle_speed > 1.5 * vf:
                        if right_distance_f > 1.2 * self.speed and right_distance_b > 1.2 * rb_vehicle_speed:
                            return True
                else:
                    if rf_vehicle_speed > 1.5 * vf:
                        if right_distance_f > 2.4 * self.speed and right_distance_b > 1.2 * rb_vehicle_speed:
                            return True