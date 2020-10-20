import os
import sys
import cv2
import glob
import time
import random
import numpy as np

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# 这里有一行注释

class GlobalVariable:
    IM_WIDTH = 640
    IM_HEIGHT = 480
    IM_FOV = 110



class CarlaDeemo:
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10)

        self.car_list = []
        self.actor_list = []
        self.world = self.client.load_world('Town02')
        self.blueprint_library = self.world.get_blueprint_library()


    def Show_camera_image(self,image):
        print(image)
        i = np.array(image.raw_data)
        i = i.reshape((GlobalVariable.IM_HEIGHT,GlobalVariable.IM_WIDTH,4))
        i = i[:, :, :3]
        cv2.imshow('',i)
        cv2.waitKey(5)


    def Genratar_a_car_with_a_camera(self):
        while True:
            try:
                coordinate = random.choice(self.world.get_map().get_spawn_points())
                car = random.choice(self.blueprint_library.filter('vehicle'))
                car = self.world.spawn_actor(car,coordinate)
                break
            except:
                pass

        car.set_autopilot(True)
        self.car_list.append(car)
        self.actor_list.append(car)

        camera = self.blueprint_library.find('sensor.camera.rgb')
        camera.set_attribute('image_size_x',str(GlobalVariable.IM_WIDTH))
        camera.set_attribute('image_size_y',str(GlobalVariable.IM_HEIGHT))
        camera.set_attribute('fov',str(GlobalVariable.IM_FOV))
        transform = carla.Transform(carla.Location(x = 2.5, z = 0.7))
        camera = self.world.spawn_actor(camera,transform,attach_to = car)
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
                car = self.world.spawn_actor(car,coordinate)
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
                x,y = map(eval,input('生成汽车位置:').split(' '))
                break
            except:
                print('坐标以空格分隔，重新输一下')

        spawn_coordinate = carla.Location(x,y)
        tolarance = eval(input('最远生成距离:'))

        exist_car_count = len(self.car_list)
        spawn_succeed_count = 0
        spawn_failed_count = 0
        new_car_list = []

        coordinates = [i for i in self.world.get_map().get_spawn_points() if i.location.distance(spawn_coordinate) < tolarance]

        for coordinate in coordinates:
            try:
                car = random.choice(self.blueprint_library.filter('vehicle'))
                car = self.world.spawn_actor(car,coordinate)
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



def main():
    carla_deemo = CarlaDeemo()

    while True:
        carla_deemo.Genratar_many_cars()
    


#if __name__ == '__main__':

#    main()