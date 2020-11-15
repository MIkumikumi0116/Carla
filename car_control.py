# 包括
# 车辆的各种控制和判断方法
import time
import random
import numpy as np
import custom_car
import carla
from carla_enviroment import GlobeVar
import sympy as sp
import math


class Car_control():
    '''汽车控制类，控制汽车的各种行为'''
    def __init__(self, car, world, car_list):
        '''初始化，将world,car_list等参数传入'''
        self.world = world
        self.car = car
        self.car_list = car_list
        self.waypoint = self.get_waypoint(self.car)
        self.lane_id = self.waypoint.lane_id
        self.road_id = self.wyapoint.road_id
        self.location = self.car.get_location()
        self.id = self.get_self_car_id()

    def acc_control(self, target_acc):
        '''控制汽车加速度'''
        self.add_imu_sensor()
        index = 0
        self.car.apply_control(
            carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))
        while True:
            if self.acc < target_acc:
                self.car.apply_control(
                    carla.VehicleControl(throttle=0.4 + index,
                                         steer=0.0,
                                         brake=0.0))
                if index + 0.4 < 1.0:
                    index += 0.01
                else:
                    index += 0.0
            elif self.get_car_acc() > target_acc:
                self.car.apply_control(
                    carla.VehicleControl(throttle=0.0,
                                         steer=0.0,
                                         brake=0.4 + index))
                if index + 0.4 < 1.0:
                    index += 0.01
                else:
                    index += 0.0
            else:
                self.car.apply_control(
                    carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))
            time.sleep(0.01)

    def road_change(self):