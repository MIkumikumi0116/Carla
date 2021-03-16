import math
import time #TODO:这样子控制加速度现在已经不行了

import carla
from globe_var import Globe_Var



class Base_Car:
    '''基础车型'''
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.iv = 20    #TODO

        self.length = self.vehicle.bounding_box.extent.x * 2

    def get_waypoint(self):
        '''获得本车的waypoint'''
        return Globe_Var.MAP.get_waypoint(self.vehicle.get_location())

    def get_distance_to_other_car(self, other_car):
        '''获得本车至other_car直线距离'''
        if other_car is None:
            return self.get_distance_to_road_end()

        location = self.vehicle.get_location()
        return location.distance(other_car.vehicle.get_location())

    def get_accelerometer(self):
        a = self.vehicle.get_acceleration()
        return pow(pow(a.x, 2) + pow(a.y, 2) + pow(a.z, 2), 0.5)

    def get_speed(self):
        v = self.vehicle.get_velocity()
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def get_distance_to_road_end(self):
        '''返回本车目前位置至道路尽头距离'''
        waypoint_list = self.get_waypoint().next_until_lane_end(Globe_Var.ROAD_WAYPOINT_STEP)
        return len(waypoint_list) * Globe_Var.ROAD_WAYPOINT_STEP

    def get_route_distanceto_other_car(self, other_car):
        '''获得本车至other_car路径距离,如果other_car为None，返回本车至该车道尽头距离'''
        return abs(self.get_distance_to_road_end() - other_car.get_distance_to_road_end()) \
                if other_car != None \
                else self.get_distance_to_road_end()

    def get_back_car(self, road_id, lane_id, distance):
        '''获得指定道路指定车道的后车，distance：本车至车道尽头距离'''
        minus = Globe_Var.FOLLOW_RANGE
        target_car = None

        for car in Globe_Var.CAR_LIST:
            if self is not car:
                waypoint = car.get_waypoint()
                target_lane_id,target_road_id = waypoint.lane_id,waypoint.road_id

                if target_road_id == road_id and target_lane_id == lane_id:
                    '''同一条道路的同一车道'''
                    target_car_distance = car.get_distance_to_road_end()

                    value = target_car_distance - distance
                    if value > 0 and value <= minus:
                        minus = value
                        target_car = car

        return target_car

    def get_forward_car(self, road_id, lane_id, distance):
        '''获得指定道路指定车道的前车，distance：本车至车道尽头距离'''
        minus = Globe_Var.FOLLOW_RANGE
        target_car = None

        for car in Globe_Var.CAR_LIST:
            if self is not car:
                waypoint = car.get_waypoint()
                target_lane_id,target_road_id = waypoint.lane_id,waypoint.road_id

                if target_road_id == road_id and target_lane_id == lane_id:
                    '''同一条道路的同一车道'''
                    target_car_distance = car.get_distance_to_road_end()

                    value = target_car_distance - distance
                    if value < 0 and -value <= minus:
                        minus = -value
                        target_car = car

        return target_car

    def get_lb_car(self):
        '''获得左车道后一辆车'''
        current_waypoint = self.get_waypoint()

        if current_waypoint.get_left_lane() == None:
            '''为1或-1则没有左边的车道'''
            return None  # 可替换成错误提示

        lane_id = current_waypoint.lane_id
        road_id = current_waypoint.road_id

        self_distance = self.get_distance_to_road_end
        lane_id = (lane_id + 1) if lane_id < 0 else (lane_id - 1)
        # 车道id由0向右呈降序，向左呈升序
        return self.get_back_car(road_id, lane_id, self_distance)

    def get_lf_car(self):
        '''获得左车道前一辆车'''
        current_waypoint = self.get_waypoint()

        if current_waypoint.get_left_lane() == None:
            '''为1或-1则没有左边的车道'''
            return None  # 可替换成错误提示

        lane_id = current_waypoint.lane_id
        road_id = current_waypoint.road_id

        self_distance = self.get_distance_to_road_end
        lane_id = (lane_id + 1) if lane_id < 0 else (lane_id - 1)
        # 车道id由0向右呈降序，向左呈升序
        return self.get_forward_car(road_id, lane_id, self_distance)

    def get_rb_car(self):
        '''获得右车道后一辆车'''
        current_waypoint = self.get_waypoint()

        if current_waypoint.get_left_lane() == None:
            '''为1或-1则没有左边的车道'''
            return None  # 可替换成错误提示

        lane_id = current_waypoint.lane_id
        road_id = current_waypoint.road_id

        self_distance = self.get_distance_to_road_end
        lane_id = (lane_id + 1) if lane_id > 0 else (lane_id - 1)
        # 车道id由0向右呈降序，向左呈升序
        return self.get_back_car(road_id, lane_id, self_distance)

    def get_rf_car(self):
        '''获得右车道前一辆车'''
        current_waypoint = self.get_waypoint()

        if current_waypoint.get_left_lane() == None:
            '''为1或-1则没有左边的车道'''
            return None  # 可替换成错误提示

        lane_id = current_waypoint.lane_id
        road_id = current_waypoint.road_id

        self_distance = self.get_distance_to_road_end
        lane_id = (lane_id + 1) if lane_id > 0 else (lane_id - 1)
        # 车道id由0向右呈降序，向左呈升序
        return self.get_forward_car(road_id, lane_id, self_distance)

    def get_next_vehicle(self):
        '''获得本车道前一辆车'''
        waypoint = self.get_waypoint()
        road_id,lane_id = waypoint.road_id,waypoint.lane_id

        distance = self.get_distance_to_road_end()
        return self.get_forward_car(road_id, lane_id, distance)

    def get_last_vehicle(self):
        '''获得本车道后一辆车'''
        waypoint = self.get_waypoint()
        road_id,lane_id = waypoint.road_id,waypoint.lane_id

        distance = self.get_distance_to_road_end()
        return self.get_back_car(road_id, lane_id, distance)

    def get_forward_list(self, road_id, lane_id, distance, range):
        '''求前方的所有车辆及跟车距离'''
        minus = Globe_Var.FOLLOW_RANGE
        List = []
        for car in Globe_Var.CAR_LIST.values():
            if self is not car:
                waypoint = car.get_waypoint()
                target_lane_id,target_road_id = waypoint.lane_id,waypoint.road_id

                if target_road_id == road_id and target_lane_id == lane_id:
                    '''同一条道路的同一车道'''
                    target_car_distance = car.get_distance_to_road_end()

                    value = target_car_distance - distance
                    if value < 0 and -value <= range:
                        List.append((car, value))
        return List

    def get_follow_list(self, road_id, lane_id, distance, range):
        '''求后方的所有车辆及跟车距离'''
        minus = Globe_Var.FOLLOW_RANGE
        List = []
        for car in Globe_Var.CAR_LIST.values():
            if self is not car:
                waypoint = car.get_waypoint()
                target_lane_id,target_road_id = waypoint.lane_id,waypoint.road_id

                if target_road_id == road_id and target_lane_id == lane_id:
                    '''同一条道路的同一车道'''
                    target_car_distance = car.get_distance_to_road_end()

                    value = target_car_distance - distance
                    if value > 0 and abs(value) <= range:
                        List.append((car, value))
        return List

    def acc_control(self):
        '''控制汽车加速度'''
        index = 0
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))

        if abs(self.get_accelerometer() - self.target_acc) < Globe_Var.ACC_TORRANCE:
            return

        if self.get_accelerometer() < self.target_acc:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.4 + index,steer=0.0,brake=0.0))
            if index + 0.4 < 1.0:
                index += 0.01
            else:
                index += 0.0

        elif self.get_accelerometer() > self.target_acc:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.4 + index,steer=0.0,brake=0.0))
            if 0.4 - index < 1.0:
                index -= 0.01
            else:
                index -= 0.0

    def velocity_control(self, target_speed):
        "控制车辆的速度"
        index = 0
        # 控制油门刹车
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))

        if abs(self.get_speed() - target_speed) < Globe_Var.SPEED_TORRANCE:
            return

        if self.get_speed() < target_speed:
            self.apply_control(carla.VehicleControl(throttle=0.4 + index,steer=0.0,brake=0.0))
            if index + 0.4 < 1.0:
                index += 0.1
            else:
                index += 0.0

        elif self.get_speed() > target_speed:
            self.apply_control(carla.VehicleControl(throttle=0.0,steer=0.0,brake=0.4 + index))
            if index + 0.4 < 1.0:
                index += 0.1
            else:
                index += 0.0

    def change_lane(self, lane_change_type):
        if lane_change_type is None:
            print('转向条件不成立！')
            return None
        elif lane_change_type == 'Left':
            self.vehicle.set_transform(self.get_waypoint().get_left_lane().transform)
        elif lane_change_type == 'Right':
            self.vehicle.set_transform(self.get_waypoint().get_right_lane().transform)