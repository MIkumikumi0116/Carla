import random
import numpy as np
from datetime import datetime

from base_car import Base_Car
from globe_var import Globe_Var



class Conventional_Car(Base_Car):
    def __init__(self, vehicle, iv):
        Car.__init__(vehicle, iv)
        self.vehicle = vehicle
        self.iv = iv
        # 传统车没有传感器
        # self.collision_sensor = None
        # self.lane_invasion_sensor = None
        # self.gnss_sensor = None
        # self.imu_sensor = None
        # self.radar_sensor = None
        # self.append_sensors()

        self.lane_change = None

    def follow_model(self):
        '''传统车跟驰模型'''
        road_id = self.get_waypoint(self.vehicle).road_id
        lane_id = self.get_waypoint(self.vehicle).lane_id
        vehicle = self.get_forward_vehicle(road_id, lane_id,
                                           self.get_waypoint_distance())
        if vehicle is None:
            print('前方没有车')   # 可用图形界面代替
            self.velocity_control(self.iv)
            return None
        car = GlobeVar.search_Car(vehicle)
        car_length = car.get_length()
        vb = self.get_speed(self.vehicle)
        # 后车速度lf.speed
        vf = car.get_speed(car.vehicle)
        # 前车速度
        d = self.get_waypoint_distance() - car.get_waypoint_distance()
        # 下面是公式转化部分
        a = -1.2
        v_p = 6.75 + 7.91 * np.tanh(0.13 * (d - self.get_length()) - 1.57)
        s_d = max(
            1 + car_length + self.speed * 1.6 + pow(vb, 2) / a / 2 -
            pow(vf, 2) / a / 2, 1 + car_length)
        lam = 2 if d < 100 else 0
        acc = 0.41 * (v_p - vb) + lam * (1 - s_d / d)
        self.acc_control(acc)
        return acc

    def lane_change_model_left(self, vehicle):
        '''换道模型'''
        distance = self.get_waypoint_distance(vehicle, 0.5)
        vf = self.get_speed(vehicle)
        # 前车速度
        limit_speed = vehicle.get_speed_limit()
        # 道路限速
        lb_vehicle = self.get_lb_vehicle()
        # 左后方汽车
        if lb_vehicle is not None:
            # 没车设置为无穷大
            left_distance_b = self.get_waypoint_distance(lb_vehicle, 0.5)
            lb_vehicle_speed = self.get_speed(lb_vehicle)
        else:
            left_distance_b = 10000
            lb_vehicle_speed = 0
        lf_vehicle = self.get_lf_vehicle()
        if lf_vehicle is not None:
            left_distance_f = self.get_waypoint_distance(lf_vehicle, 0.5)
            lf_vehicle_speed = self.get_speed(lf_vehicle)
        else:
            left_distance_f = 10000
            lf_vehicle_speed = 1000
        '''下面是公式转换部分'''
        a = 5 * limit_speed
        if distance >= 5 * a:
            random.seed(datetime.now())
            if random.random() < 0.1:
                self.lane_change = 'Left'
                self.change_lane(self.lane_change)
                return True
        else:
            if self.get_speed(self.vehicle) >= limit_speed:
                if distance < 2.4 * self.get_speed(self.vehicle):
                    if left_distance_f > distance and left_distance_b < 1.2 * lb_vehicle_speed:
                        self.lane_change = 'Left'
                        self.change_lane(self.lane_change)
                        return True
            else:
                if lf_vehicle_speed > self.get_speed(self.vehicle):
                    if lf_vehicle_speed > 1.5 * vf:
                        if left_distance_f > 1.2 * self.get_speed(
                                self.vehicle
                        ) and left_distance_b > 1.2 * lb_vehicle_speed:
                            self.lane_change = 'Left'
                            self.change_lane(self.lane_change)
                            return True
                else:
                    if lf_vehicle_speed > 1.5 * vf:
                        if left_distance_f > 2.4 * self.get_speed(
                                self.vehicle
                        ) and left_distance_b > 1.2 * lb_vehicle_speed:
                            self.lane_change = 'Left'
                            self.change_lane(self.lane_change)
                            return True