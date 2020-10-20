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

IM_WIDTH = 1280
IM_HEIGHT = 720
IM_FOV = 110


class Carla_Enviroment:
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10)

        self.car_list = []
        self.actor_list = []
        self.world = self.client.load_world('Town02')
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

        for car in new_car_list:
            car.set_autopilot(True)
            car_waypoint = self.world.get_map().get_waypoint(
                car.get_location())
            car_lane_type = car_waypoint.lane_type
            print(str(car_lane_type))
            v = car.get_velocity()
            print('Speed:   % 15.0f km/h' %
                  (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)))
            car_extent = car.bounding_box.extent
            car_length = 2 * car_extent.x
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
        while True:
            for C1 in self.car_list:
                car_lane_id = self.get_car_waypoint_id(C1)
                count = 0
                for car in self.car_list:
                    if self.get_car_waypoint_id(
                            car) == car_lane_id:
                        count += 1
                        print(str(car_lane_id) + ':{}'.format(count))
            time.sleep(0.5)

    def get_car_waypoint_id(self,car):
        car_waypoint = self.world.get_map().get_waypoint(car.get_location())
        car_lane_id = car_waypoint.lane_id
        return car_lane_id


def main():
    
    carla_Enviroment = Carla_Enviroment()

    while True:
        carla_Enviroment.Genratar_many_cars_near_specific_coordinate()
        carla_Enviroment.get_foward_car()


if __name__ == '__main__':

    main()