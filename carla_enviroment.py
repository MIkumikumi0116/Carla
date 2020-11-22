# 包括
# CarlaEnviroment类：Carla世界、地图、蓝图库管理;
# SpawnCars:生成汽车的各方法；
# 全局变量；主程序

# Todo：在自定义车辆的构造方法中完成生成车辆及添加到演员列表等操作，调用try_spawn_actor生成车辆改为调用自定义车辆的构造方法
import os
import sys
import cv2
import glob
import time
import random
import numpy as np
import custom_car

# Carla要求先写这一坨才能import carla
try:
    sys.path.append(
        glob.glob('../carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


# 为方便所有模块调用，将所有全局变量封装到一起
class GlobeVar:
    IM_WIDTH = 640  # 摄像机横向分辨率
    IM_HEIGHT = 480  # 摄像机纵向分辨率
    IM_FOV = 110  # 摄像机视野角度
    FOLLOW_RANGE = 100  # 跟驰模型适用范围


class CarlaEnviroment:
    '''Carla世界、地图、蓝图库管理'''
    def __init__(self):
        '''连接服务器，创建世界、蓝图库、车辆列表、演员列表'''
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10)

        self.world = self.client.load_world('Town04')
        self.bp_library = self.world.get_bp_library()
        self.car_list = []
        self.actor_list = []

    def Get_Spawn_Points(self):
        '''获取所有可供生成的坐标'''
        return self.world.get_map().get_spawn_points()

    def Get_Bp(self, ID):
        '''根据ID返回蓝图'''
        return self.bp_library.filter(ID)

    def Get_Transform(self, x=0, y=0, z=0, p=0, yaw=0, r=0):
        '''获取Carla.Transform'''
        return carla.Transform(carla.Location(x, y, z),
                               carla.Rotation(p, yaw, r))

    def Add_Car(self, car):
        '''向CarlaEnviroment加入汽车'''
        self.car_list.append(car)
        self.actor_list.append(car)

    def Add_Actor(self, actor):
        '''向CarlaEnviroment加入演员'''
        self.actor_list.append(actor)

    def Destory_All_Actor(self):
        '''销毁所有演员'''
        for actor in self.actor_list:
            actor.destory()

        self.car_list = []
        self.actor_list = []


class UserInterface:
    '''一些交互接口,TODO：后期改用GUI实现'''
    def Show_camera_image(image):
        '''若要展示摄像机的图像，届时用Qt实现'''
        pass


class SpawnCars:
    '''各种生成车辆的方法，TODO:输入信息和输出提示信息后期整合到UserInterface里'''
    def Spawn_car_with_camera(self):
        '''生成一辆带摄像头的汽车'''
        #尝试生成车辆直到生成成功
        while True:
            car_bp = random.choice(CarlaEnviroment.Get_Bp('vehicle'))
            car_transform = random.choice(CarlaEnviroment.Get_Spawn_Points())
            cam_transform = CarlaEnviroment.Get_Transform(x=2.5, z=0.7)
            car = custom_car.CarWithCarema(car_bp, car_transform,
                                           cam_transform)
            if car == NULL:
                continue
            break

        car.set_autopilot(True)

    def Genratar_cars(self):
        '''尝试生成指定数量的车辆'''
        count = eval(input('生成汽车数量:'))
        exist_car_count = len(CarlaEnviroment.car_list)
        spawn_succeed_count = 0  #这俩用于输出提示信息
        spawn_failed_count = 0
        new_car_list = [
        ]  #由于不明原因，生成汽车同时启用自动驾驶会使自动驾驶不起作用，因此在new_car_list暂存新生成的汽车，后面再依次启用自动驾驶

        coordinates = CarlaEnviroment.Get_Spawn_Points()

        for coordinate in coordinates:
            car_bp = random.choice(CarlaEnviroment.Get_Bp('vehicle'))
            car = custom_car.CustomCar(car_bp, coordinate)
            #生成失败，进入下次循环，生成成功，跳出循环
            if car == NULL:
                spawn_failed_count += 1
                continue

            new_car_list.append(car)
            spawn_succeed_count += 1
            if spawn_succeed_count == count:
                break

        for car in new_car_list:
            car.set_autopilot(True)

        print('\n')
        print('原有汽车数量：' + str(exist_car_count))
        print('可供生成坐标数量：' + str(len(coordinates)))
        print('生成成功汽车数量：' + str(spawn_succeed_count))
        print('生成失败汽车数量：' + str(count - spawn_succeed_count))
        print('现有汽车数量：' + str(len(CarlaEnviroment.car_list)))
        print('生成失败次数：' + str(spawn_failed_count))
        print('\n')

    def Genratar_cars_near_position(self):
        '''尝试在指定范围生成指定数量的车辆'''
        count = eval(input('生成汽车数量:'))

        while True:
            try:
                x, y = map(eval, input('生成汽车位置:').split(' '))
                spawn_coordinate = carla.Location(x, y)
                break
            except:
                print('坐标以空格分隔，重新输一下')

        tolerance = eval(input('最远生成距离:'))

        exist_car_count = len(CarlaEnviroment.car_list)
        spawn_succeed_count = 0
        spawn_failed_count = 0
        new_car_list = []

        # 选取符合条件的坐标
        coordinates = [
            i for i in CarlaEnviroment.Get_Spawn_Points()
            if i.location.distance(spawn_coordinate) <= tolerance
        ]

        for coordinate in coordinates:
            car_bp = random.choice(CarlaEnviroment.Get_Bp('vehicle'))
            car = custom_car.CustomCar(car_bp, coordinate)
            # 生成失败，进入下次循环，生成成功，跳出循环
            if car == NULL:
                spawn_failed_count += 1
                continue

            new_car_list.append(car)
            spawn_succeed_count += 1
            if spawn_succeed_count == count:
                break

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


# def main():
#     '''主程序，需要测试啥就往里面写啥'''
#     carla_enviroment = CarlaEnviroment()

#     while True:
#         carla_enviroment.Genratar_cars()


#if __name__ == '__main__':

#main()
