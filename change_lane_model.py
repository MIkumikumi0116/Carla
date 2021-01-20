# 车的各种模型
from datetime import datetime
import random
import numpy as np
import carla
import sympy as sp
import math
from car_control import Car_control
from follow_model import Follow_model


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
        self.control = Car_control(car)
        self.lane_change = None
        self.lane_state = False

    def launch_lane_change(self):
        '''装载换道模型'''
        self.set_state(True)
        if self.lane_change_model_right(self.car.vehicle):
            print("向右变道成功！")
            return True
        else:
            print("向右变道失败！")
            return False
        if self.lane_change_model_left(self.car.vehicle):
            print("向左变道成功！")
            return True
        else:
            print("向左变道失败！")
            return False

    def close_model(self, lane_state):
        '''关闭模型'''
        self.set_state(False)

    def set_state(self, index):
        '''设置模型状态'''
        self.lane_state = index

    def lane_change_model_left(self, vehicle):
        '''换道模型'''
        distance = self.car.get_waypoint_distance(vehicle, 0.5)
        vf = self.car.get_speed(vehicle.get_velocity())
        # 前车速度
        limit_speed = vehicle.get_speed_limit()
        # 道路限速
        lb_vehicle = self.lb_vehicle
        if lb_vehicle is not None:
            # 没车设置为无穷大
            left_distance_b = self.car.get_waypoint_distance(lb_vehicle, 0.5)
            lb_vehicle_velocity = lb_vehicle.get_velocity()
            lb_vehicle_speed = self.car.get_speed(lb_vehicle_velocity)
        else:
            left_distance_b = 10000
            lb_vehicle_speed = 0
        lf_vehicle = self.lf_vehicle
        if lf_vehicle is not None:
            left_distance_f = self.car.get_waypoint_distance(lf_vehicle, 0.5)
            lf_vehicle_velocity = lf_vehicle.get_velocity()
            lf_vehicle_speed = self.car.get_speed(lf_vehicle_velocity)
        else:
            left_distance_f = 10000
            lf_vehicle_speed = 1000
        '''下面是公式转换部分'''
        a = 5 * limit_speed
        if distance >= 5 * a:
            random.seed(datetime.now())
            if random.random() < 0.1:
                self.lane_change = 'Left'
                self.control.change_lane(self.lane_change)
                return True
        else:
            if self.speed >= limit_speed:
                if distance < 2.4 * self.speed:
                    if left_distance_f > distance and left_distance_b < 1.2 * lb_vehicle_speed:
                        self.lane_change = 'Left'
                        self.control.change_lane(self.lane_change)
                        return True
            else:
                if lf_vehicle_speed > self.speed:
                    if lf_vehicle_speed > 1.5 * vf:
                        if left_distance_f > 1.2 * self.speed and left_distance_b > 1.2 * lb_vehicle_speed:
                            self.lane_change = 'Left'
                            self.control.change_lane(self.lane_change)
                            return True
                else:
                    if lf_vehicle_speed > 1.5 * vf:
                        if left_distance_f > 2.4 * self.speed and left_distance_b > 1.2 * lb_vehicle_speed:
                            self.lane_change = 'Left'
                            self.control.change_lane(self.lane_change)
                            return True

    def lane_change_model_right(self, vehicle):
        '''换道模型'''
        distance = self.car.get_waypoint_distance(vehicle, 0.5)
        vf = self.car.get_speed(vehicle.get_velocity())
        # 前车速度
        limit_speed = vehicle.get_speed_limit()
        # 道路限速
        rb_vehicle = self.rb_vehicle
        if rb_vehicle is not None:
            right_distance_b = self.car.get_waypoint_distance(rb_vehicle, 0.5)
            rb_vehicle_velocity = rb_vehicle.get_velocity()
            rb_vehicle_speed = self.car.get_speed(rb_vehicle_velocity)
        else:
            right_distance_b = 10000
            rb_vehicle_speed = 0
        rf_vehicle = self.rf_vehicle
        if rf_vehicle is not None:
            right_distance_f = self.car.get_waypoint_distance(rf_vehicle, 0.5)
            rf_vehicle_velocity = rf_vehicle.get_velocity()
            rf_vehicle_speed = self.car.get_speed(rf_vehicle_velocity)
        else:
            right_distance_f = 10000
            rf_vehicle_speed = 1000
        '''下面是公式转换部分'''
        a = 5 * limit_speed
        if distance >= 5 * a:
            random.seed(datetime.now())
            if random.random() < 0.1:
                self.lane_change = 'Right'
                self.control.change_lane(self.lane_change)
                return True
        else:
            if self.speed >= limit_speed:
                if distance < 2.4 * self.speed:
                    if right_distance_f > distance and right_distance_b < 1.2 * rb_vehicle_speed:
                        self.lane_change == 'Right'
                        self.control.change_lane(self.lane_change)
                        return True
            else:
                if rf_vehicle_speed > self.speed:
                    if rf_vehicle_speed > 1.5 * vf:
                        if right_distance_f > 1.2 * self.speed and right_distance_b > 1.2 * rb_vehicle_speed:
                            self.lane_change == 'Right'
                            self.control.change_lane(self.lane_change)
                            return True
                else:
                    if rf_vehicle_speed > 1.5 * vf:
                        if right_distance_f > 2.4 * self.speed and right_distance_b > 1.2 * rb_vehicle_speed:
                            self.lane_change == 'Right'
                            self.control.change_lane(self.lane_change)
                            return True

    def idm_change_left(self, As, AFV, APFV):
        sm = 0
        psm = 0
        for item in AFV:
            sm += item[0] - item[1]
        for item in APFV:
            if item[0] >= -2:
                psm += item[0] - item[1]
            else:
                return False
        usv = As[0] - As[1] + 0.1 * (sum + psm)
        if As[0] < -2:
            return False
        if usv <= 0.3:
            return False
        self.lane_state = 'Left'
        return True
