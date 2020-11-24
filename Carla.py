import os
import sys
import cv2
import glob
import time
import random
import numpy as np
import math

try:
    sys.path.append(
        glob.glob('../carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from Car import Car

IM_WIDTH = 1280
IM_HEIGHT = 720
IM_FOV = 110


class Carla_Enviroment:
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10)

        self.car_list = []
        self.actor_list = []
        self.world = self.client.load_world('Town04')
        self.blueprint_library = self.world.get_blueprint_library()

    def Show_camera_image(self, image):
        i = np.array(image.raw_data)
        i = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
        i = i[:, :, :3]
        cv2.imshow('', i)
        cv2.waitKey(1)

    def Genratar_a_car_with_a_camera(self):
        while True:
            try:
                coordinate = random.choice(
                    self.world.get_map().get_spawn_points())
                car = random.choice(self.blueprint_library.filter('vehicle'))
                car = self.world.spawn_actor(car, coordinate)
                break
            except:
                pass

        car.set_autopilot(True)
        self.car_list.append(car)
        self.actor_list.append(car)

        camera = self.blueprint_library.find('sensor.camera.rgb')
        camera.set_attribute('image_size_x', str(IM_WIDTH))
        camera.set_attribute('image_size_y', str(IM_HEIGHT))
        camera.set_attribute('fov', str(IM_FOV))
        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        camera = self.world.spawn_actor(camera, transform, attach_to=car)
        self.actor_list.append(camera)

        camera.listen(lambda image: self.Show_camera_image(image))

    def Genratar_many_cars(self):
        count = eval(input('生成汽车数量:'))
        exist_car_count = len(self.car_list)
        spawn_succeed_count = 0
        spawn_failed_count = 0
        new_car_list = []

        coordinates = self.world.get_map().get_spawn_points()

        for coordinate in coordinates:
            try:
                car = random.choice(self.blueprint_library.filter('vehicle'))
                car = self.world.spawn_actor(car, coordinate)
                self.car_list.append(car)
                self.actor_list.append(car)
                new_car_list.append(car)
                spawn_succeed_count += 1

                if spawn_succeed_count == count:
                    break
            except:
                spawn_failed_count += 1

        for car in new_car_list:
            car.set_autopilot(True)
            car_waypoint = car.get_waypoint()
            car.lane_type = car_waypoint.lane_type
            print(str(car.lane_type))

        print('\n')
        print('原有汽车数量：' + str(exist_car_count))
        print('可供生成坐标数量：' + str(len(coordinates)))
        print('生成成功汽车数量：' + str(spawn_succeed_count))
        print('生成失败汽车数量：' + str(count - spawn_succeed_count))
        print('现有汽车数量：' + str(len(self.car_list)))
        print('生成失败次数：' + str(spawn_failed_count))
        print('\n')

    def Genratar_many_cars_near_specific_coordinate(self):
        count = eval(input('生成汽车数量:'))

        while True:
            try:
                x, y = map(eval, input('生成汽车位置:').split(' '))
                break
            except:
                print('坐标以空格分隔，重新输一下')

        spawn_coordinate = carla.Location(x, y)
        tolarance = eval(input('最远生成距离:'))

        exist_car_count = len(self.car_list)
        spawn_succeed_count = 0
        spawn_failed_count = 0
        new_car_list = []

        coordinates = [
            i for i in self.world.get_map().get_spawn_points()
            if i.location.distance(spawn_coordinate) < tolarance
        ]

        for coordinate in coordinates:
            try:
                car = random.choice(self.blueprint_library.filter('vehicle'))
                car = self.world.spawn_actor(car, coordinate)
                self.car_list.append(car)
                self.actor_list.append(car)
                new_car_list.append(car)
                spawn_succeed_count += 1

                if spawn_succeed_count == count:
                    break
            except:
                spawn_failed_count += 1
        random.shuffle(self.car_list)

        for vehicle in new_car_list:
            vehicle.set_autopilot(True)
            car = Car(vehicle, self.world, self.car_list)
            # car_waypoint = self.world.get_map().get_waypoint(
            #     car.get_location())
            car_waypoint = car.waypoint()
            car_lane_type = car_waypoint.lane_type
            print(str(car_lane_type))
            self.output_speed(car.velocity)
            car_length = 2 * car.get_length()
            print(str(car_length))

        print('\n')
        print('原有汽车数量：' + str(exist_car_count))
        print('可供生成坐标数量：' + str(len(coordinates)))
        print('生成成功汽车数量：' + str(spawn_succeed_count))
        print('生成失败汽车数量：' + str(count - spawn_succeed_count))
        print('现有汽车数量：' + str(len(self.car_list)))
        print('生成失败次数：' + str(spawn_failed_count))
        print('\n')

    def Destory_all_actor(self):
        for actor in self.actor_list:
            actor.destory()
        self.car_list = []
        self.actor_list = []

    def get_foward_car(self):
        random.shuffle(self.car_list)
        while True:
            for i in range(len(self.car_list)):
                i_waypoint = Carla_Enviroment.get_waypoint(
                    self, self.car_list[i])
                i_lane_id = i_waypoint.lane_id
                i_road_id = i_waypoint.road_id
                i_length = len(i_waypoint.next_until_lane_end(0.5))
                min = 10000
                index = 0
                count = 1
                for j in range(len(self.car_list)):
                    j_waypoint = Carla_Enviroment.get_waypoint(
                        self, self.car_list[j])
                    j_length = len(j_waypoint.next_until_lane_end(0.5))
                    if i_road_id == j_waypoint.road_id and j_waypoint.lane_id == i_lane_id and i != j:
                        if i_length > j_length and abs(i_length -
                                                       j_length) < min:
                            min = abs(i_length - j_length)
                            index = j
                        count += 1
                print(
                    str(i_road_id) + '号道路的' + str(i_lane_id) + '号车道' +
                    '共:{}辆车'.format(count))
                if min != 10000:
                    print('{}'.format(i) + '号车的前车是' + '{}'.format(index) +
                          '号车')
                    print('两车的距离为{}'.format(min * 2) + '米')
                else:
                    print('{}'.format(i) + '号车为该车道第一辆车')
            time.sleep(0.3)

    '''
    def get_waypoint_lane_id(self, car):
        car_waypoint = self.world.get_map().get_waypoint(car.get_location())
        car_lane_id = car_waypoint.lane_id
        return car_lane_id
    '''

    def output_speed(self, v):
        print('Speed:   % 15.0f km/h' %
              (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)))

    def get_waypoint(self, car):
        '''获得waypoint对象'''
        return self.world.get_map().get_waypoint(car.get_location())


def main():
    carla_Enviroment = Carla_Enviroment()

    while True:
        carla_Enviroment.Genratar_many_cars_near_specific_coordinate()
        carla_Enviroment.get_foward_car()


if __name__ == '__main__':

    main()