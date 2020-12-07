# 关于Carla中的车的各项属性
import glob
import os
import sys

try:
    sys.path.append(
        glob.glob('../carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import math
from sensor_manager import CollisionSensor
from sensor_manager import IMUSensor
from sensor_manager import LaneInvasionSensor
from sensor_manager import RadarSensor
from sensor_manager import GnssSensor


class GlobeVar:
    IM_WIDTH = 1280
    IM_HEIGHT = 720
    IM_FOV = 110
    FOLLOW_RANGE = 100


class Car():
    '''车的基本属性类'''
    def __init__(self, vehicle, world, vehicle_list):
        '''初始化，将vechile,world传入'''
        self.vehicle = vehicle
        self.world = world
        self.extent = self.vehicle.bounding_box.extent
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.car_list = vehicle_list
        self.location = self.vehicle.get_location()
        self.velocity = self.vehicle.get_velocity()
        self.id = self.get_self_vehicle_id()
        self.acc = None
        self.velocity = self.vehicle.get_velocity()
        # carlist在生成时就需要先随机排序
        self.waypoint = self.get_waypoint(self.vehicle)
        self.waypoint_list = None
        self.lane_id = self.waypoint.lane_id
        self.road_id = self.waypoint.road_id
        self.append_sensors()
        self.lb_vehicle = None
        self.lf_vehicle = None
        self.rb_vehicle = None
        self.rf_vehicle = None
        self.next_vehicle = None
        self.last_vehicle = None
        self.get_nearby_vehicle()

    def get_waypoint(self, vehicle):
        '''获得waypoint对象'''
        return self.world.get_map().get_waypoint(vehicle.get_location())

    def get_waypoint_list(self, waypoint, index):
        '''返回航点所得距离'''
        return waypoint.next_until_lane_end(index)

    def get_distance(self, vehicle):
        '''返回直线距离'''
        return self.location.distance(vehicle)

    def get_self_vehicle_id(self):
        '''获得本车id'''
        for i in range(len(self.car_list)):
            if self.vehicle.get_location() == self.location:
                return i

    def get_car_list(self, world):
        '''获得已存在的车辆列表'''
        return world.get_actors().filter('vehicle.*')

    def get_accelerometer(self):
        a_x = self.imu_sensor.accelerometer.x
        a_y = self.imu_sensor.accelerometer.y
        a_z = self.imu_sensor.accelerometer.z
        return pow(pow(a_x, 2) + pow(a_y, 2) + pow(a_z, 2), 0.5)

    def get_speed(velocity):
        '''获取速度'''
        v = velocity
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def get_length(self):
        '''返回本长度'''
        return self.extent.x * 2

    def append_sensors(self):
        '''加载传感器'''
        self.collision_sensor = CollisionSensor(self.vehicle)
        self.lane_invasion_sensor = LaneInvasionSensor(self.vehicle)
        self.gnss_sensor = GnssSensor(self.vehicle)
        self.imu_sensor = IMUSensor(self.vehicle)

    def get_nearby_vehicle(self):
        '''获得本车周围车道的车辆,若返回-1则说明无满足条件的车'''
        self.lb_vehicle = self.get_lb_vehicle()
        self.lf_vehicle = self.get_lf_vehicle()
        self.rb_vehicle = self.get_rb_vehicle()
        self.rf_vehicle = self.get_rf_vehicle()
        self.next_vehicle = self.get_next_vehicle()
        self.last_vehicle = self.get_last_vehicle()

    def get_waypoint_distance(self, vehicle, index):
        '''求距离'''
        self.waypoint_list = self.get_waypoint_list(self.waypoint, index)
        v_waypoint = self.get_waypoint(vehicle)
        v_length = self.get_waypoint_list(v_waypoint, 0.5)
        return index * (abs(v_length - self.waypoint_list))

    def get_lb_vehicle(self):
        '''获得左车道后一辆车'''
        if abs(self.lane_id) == 1:
            '''为1或-1则没有左边的车道'''
            return None
        my_length = len(self.get_waypoint_list(self.waypoint, 0.5))
        my_lane_id = self.lane_id + 1 if self.lane_id < 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_back_vehicle(self.road_id, my_lane_id, my_length)

    def get_lf_vehicle(self):
        '''获得左车道前一辆车'''
        if abs(self.lane_id) == 1:
            '''为1则不能向左换道'''
            return None
        my_length = len(self.get_waypoint_list(self.waypoint, 0.5))
        my_lane_id = self.lane_id + 1 if self.lane_id < 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_forward_vehicle(self.road_id, my_lane_id, my_length)

    def get_rb_vehicle(self):
        '''获得右车道后一辆车'''
        my_length = len(self.get_waypoint_list(self.waypoint, 0.5))
        my_lane_id = self.lane_id + 1 if self.lane_id > 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_back_vehicle(self.road_id, my_lane_id, my_length)

    def get_rf_vehicle(self):
        '''获得右车道前一辆车'''
        my_length = len(self.get_waypoint_list(self.waypoint, 0.5))
        my_lane_id = self.lane_id + 1 if self.lane_id > 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_forward_vehicle(self.road_id, my_lane_id, my_length)

    def get_back_vehicle(self, road_id, lane_id, length):
        '''求后车'''
        min = GlobeVar.FOLLOW_RANGE
        index = -1
        for i in range(len(self.car_list)):
            if i != self.id:
                i_waypoint = self.get_waypoint(self.car_list[i])
                i_length = len(i_waypoint.next_until_lane_end(0.5))
                i_road_id = i_waypoint.road_id
                i_lane_id = i_waypoint.lane_id
                if i_road_id == road_id and i_lane_id == lane_id:
                    '''同一条道路的同一车道'''
                    if length < i_length and abs(i_length - length) < min:
                        # 航点数比本车多表示该车在本车后面
                        min = abs(i_length - length)
                        index = i
        if index == -1:
            return None
        return self.car_list[index]

    def get_forward_vehicle(self, road_id, lane_id, length):
        '''求前车及其与本车的距离'''
        min = GlobeVar.FOLLOW_RANGE
        index = -1
        for i in range(len(self.car_list)):
            if i != self.id:
                i_waypoint = self.get_waypoint(self.car_list[i])
                i_length = len(
                    i_waypoint.next_until_lane_end( 0.5))
                i_road_id = i_waypoint.road_id
                i_lane_id = i_waypoint.lane_id
                if i_road_id == road_id and i_lane_id == lane_id:
                    '''同一条道路的同一车道'''
                    if length > i_length and abs(i_length - length) < min:
                        # 航点数比本车少表示该车在本车前面
                        min = abs(i_length - length)
                        index = i
        if index == -1:
            return None
        return self.car_list[index]

    def get_next_vehicle(self):
        '''获得本车道前一辆车'''
        my_length = len(self.get_waypoint_list(self.waypoint, 0.5))
        return self.get_forward_vehicle(self.road_id, self.lane_id, my_length)

    def get_last_vehicle(self):
        '''获得本车道后一辆车'''
        my_length = len(self.get_waypoint_list(self.waypoint, 0.5))
        return self.get_back_vehicle(self.road_id, self.lane_id, my_length)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.vehicle)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None