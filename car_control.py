# 包括
# 车辆的各种控制和判断方法
import time
import random
import numpy as np

import carla


class Car_control():
    '''汽车控制类，控制汽车的各种行为'''
    def __init__(self, car):
        '''初始化，将world,car_list等参数传入'''
        self.world = car.world
        self.car = car
        self.speed = car.get_speed(self.car.velocity)
        self.acc = self.car.get_accelerometer()
        self.vehicle_list = self.car.vehicle_list
        self.waypoint = self.car.get_waypoint(self.car.vehicle)
        self.lane_change = self.waypoint.lane_change
        self.lane_id = self.car.lane_id
        self.road_id = self.car.road_id
        self.location = self.car.location
        self.id = self.car.id

    def acc_control(self, target_acc):
        '''控制汽车加速度'''
        index = 0
        self.car.vehicle.apply_control(
            carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))
        while True:
            if self.acc < target_acc:
                self.car.vehicle.apply_control(
                    carla.VehicleControl(throttle=0.4 + index,
                                         steer=0.0,
                                         brake=0.0))
                if index + 0.4 < 1.0:
                    index += 0.01
                else:
                    index += 0.0
            elif self.acc > target_acc:
                self.car.vehicle.apply_control(
                    carla.VehicleControl(throttle=0.4 + index,
                                         steer=0.0,
                                         brake=0.0))
                if 0.4 - index < 1.0:
                    index -= 0.01
                else:
                    index -= 0.0
            else:
                self.car.vehicle.apply_control(
                    carla.VehicleControl(throttle=4 + index,
                                         steer=0.0,
                                         brake=0.0))
            time.sleep(0.01)

    def velocity_control(self, speed):
        "控制车辆的速度"
        index = 0
        self.car.vehicle.apply_control(
            carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))
        while True:
            if self.speed < speed:
                self.car.vehicle.apply_control(
                    carla.VehicleControl(throttle=0.4 + index,
                                         steer=0.0,
                                         brake=0.0))
                if index + 0.4 < 1.0:
                    index += 0.1
                else:
                    index += 0.0
            elif self.speed > speed:
                self.car.vehicle.apply_control(
                    carla.VehicleControl(throttle=0.0,
                                         steer=0.0,
                                         brake=0.4 + index))
                if index + 0.4 < 1.0:
                    index += 0.1
                else:
                    index += 0.0
            else:
                self.car.vehicle.apply_control(
                    carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))
            time.sleep(0.01)

    def change_lane(self, change_type):
        if change_type is None:
            return None
        if change_type == 'Left':
            self.car.vehicle.set_transform(
                self.car.waypoint.get_left_lane().transform)
        if change_type == 'Right':
            self.car.vehicle.set_transform(
                self.car.waypoint.get_right_lane().transform)
