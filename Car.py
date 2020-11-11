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

from carla_enviroment import GlobeVar
import math
from sensor_manager import CollisionSensor
from sensor_manager import IMUSensor
from sensor_manager import LaneInvasionSensor
from sensor_manager import RadarSensor
from sensor_manager import GnssSensor


class Car():
    '''车的基本属性类'''
    def __init__(self, car, world):
        '''初始化，将vechile,world传入'''
        self.car = car
        self.id = self.get_self_car_id()
        self.extent = car.extent
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.acc = None
        self.car_list = self.get_car_list(world)
        # carlist在生成时就需要先随机排序
        self.waypoint = self.get_waypoint(self.car)
        self.lane_id = self.waypoint.lane_id
        self.road_id = self.wyapoint.road_id
        self.append_sensors()
        self.location = self.car.get_location()
        self.velocity = self.car.get_velocity()
        self.lb_car = None
        self.lf_car = None
        self.rb_car = None
        self.rf_car = None
        self.next_car = None
        self.last_car = None
        self.get_nearby_car()

    def get_waypoint(self, car):
        '''获得waypoint对象'''
        return self.world.get_map().get_waypoint(car.get_location())

    def get_self_car_id(self):
        '''获得本车id'''
        for i in range(len(self.car_list)):
            if self.get_location() == self.location:
                return i

    def get_car_list(self, world):
        '''获得已存在的车辆列表'''
        return world.get_actors().filter('vehicle.*')

    def get_speed(velocity):
        '''获取速度'''
        v = velocity
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def append_sensors(self):
        '''加载传感器'''
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)

    def get_nearby_car(self):
        '''获得本车周围车道的车辆,若返回-1则说明无满足条件的车'''
        self.lb_car = self.get_lb_car()
        self.lf_car = self.get_lf_car()
        self.rb_car = self.get_rb_car()
        self.rf_car = self.get_rf_car()
        self.next_car = self.get_next_car()
        self.last_car = self.get_last_car()

    def get_lb_car(self):
        '''获得左车道后一辆车'''
        if abs(self.lane_id) == 1:
            '''为1或-1则没有左边的车道'''
            return -1
        my_length = len(self.waypoint.next_until_lane_end(0.5))
        my_lane_id = self.lane_id + 1 if self.lane_id < 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_back_car(self.road_id, my_lane_id, my_length)

    def get_lf_car(self):
        '''获得左车道前一辆车'''
        if abs(self.lane_id) == 1:
            '''为1则不能向左换道'''
            return -1
        my_length = len(self.waypoint.next_until_lane_end(0.5))
        my_lane_id = self.lane_id + 1 if self.lane_id < 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_forward_car(self.road_id, my_lane_id, my_length)

    def get_rb_car(self):
        '''获得右车道后一辆车'''
        my_length = len(self.waypoint.next_until_lane_end(0.5))
        my_lane_id = self.lane_id + 1 if self.lane_id > 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_back_car(self.road_id, my_lane_id, my_length)

    def get_rf_car(self):
        '''获得右车道前一辆车'''
        my_length = len(self.waypoint.next_until_lane_end(0.5))
        my_lane_id = self.lane_id + 1 if self.lane_id > 0 else self.lane_id - 1
        # 车道id由0向右呈降序，向左呈升序
        return self.get_forward_car(self.road_id, my_lane_id, my_length)

    def get_back_car(self, road_id, lane_id, length):
        '''求后车'''
        min = GlobeVar.FOLLOW_RANGE
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
        if index == -1:
            return -1
        return self.car_list[index]

    def get_forward_car(self, road_id, lane_id, length):
        '''求前车及其与本车的距离'''
        min = GlobeVar.FOLLOW_RANGE
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
        if index == -1:
            return -1
        return self.car_list[index]

    def get_next_car(self):
        '''获得本车道前一辆车'''
        my_length = len(self.waypoint.next_until_lane_end(0.5))
        return self.get_forward_car(self.road_id, self.lane_id, my_length)

    def get_last_car(self):
        '''获得本车道后一辆车'''
        my_length = len(self.waypoint.next_until_lane_end(0.5))
        return self.get_back_car(self.road_id, self.lane_id, my_length)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.car)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None