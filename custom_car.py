#包括：所有的自定义车辆

import carla
from carla_enviroment import GlobeVar
from carla_enviroment import UserInterface
from carla_enviroment import CarlaEnviroment


class CustomCar:
    '''所有自定义车辆的基类'''
    def __init__(self, bp, transform):
        '''依据蓝图和位置生成车辆'''
        self.car = CarlaEnviroment.world.try_spawn_actor(bp, transform)
        if self.car == NULL:
            return NULL

        CarlaEnviroment.Add_Car(self.car)
        return self.car

    def DrivingModel(self):
        '''虚函数，由各派生类实现'''
        pass

    def Mount_RGB_Cam(self, transform):
        '''安装RGB摄像机'''
        RGB_cam_bp = CarlaEnviroment.Get_Bp('sensor.camera.rgb')
        RGB_cam_bp.set_attribute('image_size_x', str(GlobeVar.IM_WIDTH))
        RGB_cam_bp.set_attribute('image_size_y', str(GlobeVar.IM_HEIGHT))
        RGB_cam_bp.set_attribute('fov', str(GlobeVar.IM_FOV))

        self.RGB_cam = world.spawn_actor(RGB_cam_bp,
                                         transform,
                                         attach_to=self.car)
        self.RGB_cam.listen(
            lambda image: UserInterface.Show_camera_image(image))
        CarlaEnviroment.Add_Actor(self.RGB_cam)

        self.RGB_cam_received_data = NULL
        #self.RGB_cam_received_data_copy = NULL   详情见：每零点一秒执行一次的函数(self)
        self.RGB_cam.listen(
            lambda image: CarDemo.On_RGB_Cinema_Received_Data(image))

    def On_RGB_Cam_Received_Data(self, image):
        '''虚函数，由各派生类实现'''
        pass

    def Mount_Collision_Detector(self):
        '''安装碰撞探测器'''
        collision_detector_bp = CarlaEnviroment.Get_Bp(
            'sensor.other.collision')
        transform = CarlaEnviroment.Get_Transform(x=0, z=0)

        self.collision_detector = CarlaEnviroment.world.spawn_actor(
            collision_detector_bp, transform, attach_to=self.car)
        CarlaEnviroment.Add_Actor(self.collision_detector)

        self.collision_detector_received_data = NULL
        #self.collision_detector_received_data_copy = NULL
        self.collision_detector.listen(
            lambda collision_event: self.On_Collision_Detector_Received_Data(
                collision_event))

    def On_Collision_Detector_Received_Data(self, collision_event):
        '''虚函数，由各派生类实现'''
        pass

    def Mount_Obstacle_Detector(self):
        '''安装障碍探测器'''
        obstacle_detector_bp = CarlaEnviroment.Get_Bp('sensor.other.obstacle')
        transform = CarlaEnviroment.Get_Transform(x=0, z=0)

        self.obstacle_detector = CarlaEnviroment.world.spawn_actor(
            obstacle_detector_bp, transform, attach_to=self.car)
        CarlaEnviroment.Add_Actor(self.obstacle_detector)

        self.obstacle_detector_received_data = NULL
        #self.obstacle_detector_received_data_copy = NULL
        self.obstacle_detector.listen(
            lambda obstacle_detection_event: self.
            On_Obstacle_Detector_Received_Data(obstacle_detection_event))

    def On_Obstacle_Detector_Received_Data(self, obstacle_detection_event):
        '''虚函数，由各派生类实现'''
        pass


class CarWithCarema(CustomCar):
    def __init__(self, bp, car_transform, cam_transform):
        '''生成汽车，并安装RGB摄像机'''
        self.car = CarlaEnviroment.world.try_spawn_actor(bp, transform)
        if self.car == NULL:
            return NULL

        CarlaEnviroment.Add_Car(self.car)

        CustomCar.Mount_RGB_Cam(cam_transform)

        return self.car

    def On_RGB_Cam_Received_Data(self, image):
        UserInterface.Show_camera_image(image)
        pass


class CarDemo(CustomCar):
    '''示例：安装RGB摄像机、碰撞探测器、障碍探测器，有行驶模型的车'''
    def __init__(self, bp, transform):
        '''生成汽车，并安装RGB摄像机、碰撞探测器、障碍探测器'''
        self.car = CarlaEnviroment.world.try_spawn_actor(bp, transform)
        if self.car == NULL:
            return NULL

        CarlaEnviroment.Add_Car(self.car)

        CustomCar.Mount_RGB_Cam()
        CustomCar.Mount_Collision_Detector()
        CustomCar.Mount_Obstacle_Detector()

        return self.car

    def On_RGB_Cam_Received_Data(self, image):
        self.RGB_cinema_received_data = image
        pass

    def On_Collision_Detector_Received_Data(self, collision_event):
        self.collision_detector_received_data = collision_event
        pass

    def On_Obstacle_Detector_Received_Data(self, obstacle_detection_event):
        self.obstacle_detector_received_data = obstacle_detection_event
        pass

    def DrivingModel(self):
        '''这里面的每一步都是大坑，可能需要写好几个函数，分别负责一步，再缝到一起组成运行原理'''
        #将各个传感器的received_data拷贝到各种的received_data_copy，防止运算过程中传感器返回的数据发生变化导致出错，
        #本步骤及received_data_copy可依实际运行情况决定是否可以删去

        #依据传感器返回的数据选择适用的模型，如跟驰模型，换道模型等等

        #将传感器返回的数据转换成模型所能接收的输入

        #依据模型及输入计算输出

        #将模型输出转换为油门力度，刹车力度，方向盘角度等能直接用于控制汽车的数据

        #应用车辆控制
        pass
