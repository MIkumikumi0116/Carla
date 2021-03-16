import time
import random

import carla

from intelligent_car import Intelligent_Car
from conventional_car import Conventional_Car
from globe_var import Globe_Var



class Master_Control:
    def __init__(self):
        self.IDEAL_SPEED = 40
        self.INTEL_CAR_RATIO = 1   #1：全生成智能车，0：全生成非智能车，小数：依概率生成
        self.ROAD_WAYPOINT_STEP = 0.5
        self.ACC_TORRANCE = 1
        self.SPEED_TORRANCE = 4
        self.FOLLOW_RANGE = 100
        self.IDM_RANGE = 300

        self.CLIENT = carla.Client('localhost', 2000)
        self.CLIENT.set_timeout(20)
        self.CAR_LIST = []

        #xodr_file = open('Carla2.0/map.xodr')
        #xodr_data = xodr_file.read()
        #self.WORLD = self.CLIENT.generate_opendrive_world(xodr_data)

        self.WORLD = self.CLIENT.load_world('Town04')
        self.MAP = self.WORLD.get_map()
        self.BP_LIBRARY = self.WORLD.get_blueprint_library()

        Globe_Var.WORLD = self.WORLD
        Globe_Var.MAP = self.MAP
        Globe_Var.BP_LIBRARY = self.BP_LIBRARY
        Globe_Var.CAR_LIST = self.CAR_LIST

    def generate_cars(self):
        '''生成汽车的唯一入口'''
        count = eval(input('生成汽车数量:'))

        while True:
            try:
                x, y = map(eval, input('生成汽车位置:').split(' '))
                break
            except:
                print('坐标以空格分隔，重新输一下')

        spawn_center_coordinate = carla.Location(x, y)
        tolarance = eval(input('最远生成距离:'))

        coordinates = [i for i in self.MAP.get_spawn_points() if i.location.distance(spawn_center_coordinate) < tolarance]

        exist_car_count = len(self.CAR_LIST)
        spawn_succeed_count = 0
        spawn_failed_count = 0

        for coordinate in coordinates:
            #try:
                vehicle = random.choice(self.BP_LIBRARY.filter('vehicle'))
                vehicle = self.WORLD.spawn_actor(vehicle, coordinate)

                #if random.random() < self.INTEL_CAR_RATIO:  #生成智能车
                car = Intelligent_Car(vehicle)
                #else:                                       #生成非智能车
                #    car = Conventional_Car(vehicle,ideal_speed)

                self.CAR_LIST.append(car)
                spawn_succeed_count += 1

                if spawn_succeed_count == count:
                    break
            #except:
            #    spawn_failed_count += 1

        print('\n')
        print('原有汽车数量：' + str(exist_car_count))
        print('可供生成坐标数量：' + str(len(coordinates)))
        print('生成成功汽车数量：' + str(spawn_succeed_count))
        print('生成失败汽车数量：' + str(count - spawn_succeed_count))
        print('现有汽车数量：' + str(len(self.CAR_LIST)))
        print('生成失败次数：' + str(spawn_failed_count))
        print('\n')

    def reset_globe(self):
        Globe_Var.IDEAL_SPEED = self.IDEAL_SPEED
        Globe_Var.INTEL_CAR_RATIO = self.INTEL_CAR_RATIO
        Globe_Var.ROAD_WAYPOINT_STEP = self.ROAD_WAYPOINT_STEP
        Globe_Var.ACC_TORRANCE = self.ACC_TORRANCE
        Globe_Var.SPEED_TORRANCE = self.SPEED_TORRANCE
        Globe_Var.FOLLOW_RANGE = self.FOLLOW_RANGE
        Globe_Var.IDM_RANGE = self.IDM_RANGE

    def environment_init(self):
        for car in self.CAR_LIST:
            car.vehicle.set_autopilot(True)

        time.sleep(4)

        for car in self.CAR_LIST:
            car.vehicle.set_autopilot(False)



def main():
    master_control = Master_Control()
    master_control.generate_cars()
    master_control.environment_init()

    while True:
        time_1 = time.time()

        master_control.reset_globe()
        for car in master_control.CAR_LIST:
            car.overall_model()
        #master_control.generate_assess()

        time_2 = time.time()
        print(1000 / (int(round(time_2 * 1000)) - int(round(time_1 * 1000))))



if __name__ == '__main__':
    main()