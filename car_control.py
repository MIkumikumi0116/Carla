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


class car_control():
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

    def is_in_follow_range(self, car, waypoint):
        '''判断是否在跟驰模型范围'''
        if self.road_id == waypoint.road_id and self.lane_id == waypoint.lane_id:
            if self.is_in_range(car.get_location()):
                return True
            else:
                return False
        else:
            return False

    def get_distance(self, car):
        '''通过carla给定的距离函数求两车直线距离'''
        return self.location.distance()

    def get_waypoint(self, car):
        '''获得waypoint对象'''
        return self.world.get_map().get_waypoint(car.get_location())

    def get_self_car_id(self):
        '''获得本车id'''
        for i in range(len(self.car_list)):
            if self.get_location() == self.location:
                return i

    def get_lb_car(self):
        '''获得左车道后一辆车及其与本车的距离'''
        my_length = len(self.waypoint.next_until_lane_end(0.5))
        if abs(self.id) == 1:
            '''为1则不能向左换道'''
            return -1
        my_lane_id = self.lane_id + 1 if self.lane_id < 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_back_car(self.road_id, my_lane_id, my_length)

    def get_lf_car(self):
        '''获得左车道前一辆车及其与本车的距离'''
        my_length = len(self.waypoint.next_until_lane_end(0.5))
        if abs(self.id) == 1:
            '''为1则不能向左换道'''
            return -1
        my_lane_id = self.lane_id + 1 if self.lane_id < 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_forward_car(self.road_id, my_lane_id, my_length)

    def get_back_car(self, road_id, lane_id, length):
        '''求后车及其与本车的距离'''
        min = 10000
        index = -1
        for i in range(len(self.car_list)):
            if i != self.id:
                i_waypoint = self.get_waypoint(self.car_list[i])
                i_length = len(i_waypoint.next_until_lane_end())
                i_road_id = i_waypoint.road_id
                i_lane_id = i_waypoint.lane_id
                if i_road_id == road_id and i_lane_id == lane_id:
                    '''同一条道路的同一车道'''
                    if length < i_length and abs(i_length - length) < min:
                        # 航点数比本车多表示该车在本车后面
                        min = abs(i_length - length)
                        index = i
        distance = min * 0.5
        return self.car_list[index], distance

    def get_forward_car(self, road_id, lane_id, length):
        '''求前车及其与本车的距离'''
        min = 10000
        index = -1
        for i in range(len(self.car_list)):
            if i != self.id:
                i_waypoint = self.get_waypoint(self.car_list[i])
                i_length = len(i_waypoint.next_until_lane_end())
                i_road_id = i_waypoint.road_id
                i_lane_id = i_waypoint.lane_id
                if i_road_id == road_id and i_lane_id == lane_id:
                    '''同一条道路的同一车道'''
                    if length > i_length and abs(i_length - length) < min:
                        # 航点数比本车少表示该车在本车前面
                        min = abs(i_length - length)
                        index = i
        distance = min * 0.5
        return self.car_list[index], distance

    def is_in_range(self, distance):
        '''判断是否在限定距离内'''
        if distance <= GlobeVar.FOLLOW_RANGE:
            return True
        else:
            return False

    def road_change_model_left(self, car, distance):
        '''换道模型'''
        self.velocity = self.car.get_velocity()
        self.speed = car_control.get_speed(self.velocity)
        # 本车速度
        vf = car_control.get_speed(car.get_velocity())
        # 前车速度
        limit_speed = car.get_speed_limit()
        lb_car, left_distance_b = self.get_lb_car()
        lb_car_velocity = lb_car.get_velocity()
        lb_car_speed = car_control.get_speed(lb_car_velocity)
        lf_car, left_distance_f = self.get_lf_car()
        lf_car_velocity = lf_car.get_velocity()
        lf_car_speed = car_control.get_speed(lf_car_velocity)
        '''下面是公式转换部分'''
        a = 5 * limit_speed
        if distance >= 5 * a:
            if random.random() < 0.1:
                return True
        else:
            if self.speed >= limit_speed:
                if distance < 2.4 * self.speed:
                    if left_distance_f > distance and left_distance_b < 1.2 * lb_car_speed:
                        return True
            else:
                if lf_car_speed > self.speed:
                    if lf_car_speed > 1.5 * vf:
                        if left_distance_f > 1.2 * self.speed and left_distance_b > 1.2 * lb_car_speed:
                            return True
                else:
                    if lf_car_speed > 1.5 * vf:
                        if left_distance_f > 2.4 * self.speed and left_distance_b > 1.2 * lb_car_speed:
                            return True

    def follow_model_plus(self, car, distance):
        '''跟驰模型'''
        '''距离为主函数通过waypoint计算'''
        self.velocity = self.car.get_velocity()
        self.extent = self.car.bounding_box.extent
        # 获得本车的矩形框
        self.length = self.extent.x * 2
        # 通过矩形框获得车长
        self.acc = self.get_car_acc()
        self.speed = car_control.get_speed(self.velocity)
        car_extent = car.bounding_box.extent
        car_length = car_extent.x * 2
        vb = self.speed
        # 后车速度
        vf = car_control.get_speed(car.get_velocity())
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
        self.acc_control(acc)

    def acc_for_follow(self, car, car_acc, distance, T, alpha):
        '''获得跟驰模型加速度'''
        '''参数依次为：车，加速度，距离，敏感度，最优速度，反应速度'''
        self.velocity = self.car.get_velocity()
        self.speed = car_control.get_speed(self.velocity)
        self.extent = self.car.bounding_box.extent
        self.length = self.extent.x * 2
        self.acc = self.get_car_acc()
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

    def get_speed(velocity):
        '''获取速度'''
        v = velocity
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

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

    def add_imu_sensor(self):
        '''添加IMU传感器'''
        imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
        imu_location = carla.Location(0, 0, 0)
        imu_rotation = carla.Rotation(0, 0, 0)
        imu_transform = carla.Transform(imu_location, imu_rotation)
        imu_bp.set_attribute("sensor_tick", str(0.01))
        self.imu = self.world.spawn_actor(
            imu_bp,
            imu_transform,
            attach_to=self.car,
            attachment_type=carla.AttachmentType.Rigid)
        self.imu.listen(
            lambda imu_sensor_data: car_control.imu_callback(imu_sensor_data))

    @staticmethod
    def imu_callback(self, imu_sensor_data):
        '''IMU传感器响应函数'''
        a_x = imu_sensor_data.accelerometer.x
        a_y = imu_sensor_data.accelerometer.y
        a_z = imu_sensor_data.accelerometer.z
        '''加速度为x,y,z三个方向，需转化'''
        self.acc = pow(pow(a_x, 2) + pow(a_y, 2) + pow(a_z, 2), 0.5)
        print("IMU measure:\n" + str(imu_sensor_data) + '\n')

