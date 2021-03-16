from base_car import Base_Car
from globe_var import Globe_Var

from sensor_manage import CollisionSensor
from sensor_manage import IMUSensor
from sensor_manage import LaneInvasionSensor
from sensor_manage import RadarSensor
from sensor_manage import GnssSensor


class Intelligent_Car(Base_Car):
    def __init__(self, vehicle):
        super().__init__(vehicle)
        self.vehicle = vehicle
        self.target_acc = 0

        # 各类传感器
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.append_sensors()

        self.lane_change_type = None   # 控制转向

    def append_sensors(self):
        '''加载传感器'''
        self.collision_sensor = CollisionSensor(self.vehicle)
        self.lane_invasion_sensor = LaneInvasionSensor(self.vehicle)
        self.gnss_sensor = GnssSensor(self.vehicle)
        self.imu_sensor = IMUSensor(self.vehicle)

    def overall_model(self):
        '''控制车辆驾驶的唯一入口'''
        waypoint = self.get_waypoint()
        lane_id,road_id = waypoint.lane_id,waypoint.road_id
        front_car = self.get_forward_car(road_id,lane_id,self.get_distance_to_road_end())

        self.target_acc = self.idm_follow_model(self.get_distance_to_other_car(front_car),self,front_car)
        self.acc_control()

    def idm_follow_model(self, distance, back_car, front_car):
        '''IDM跟驰模型,跟车距离，后车，前车'''
        if front_car is None:
            return 0

        front_car_length = front_car.length
        # 后车速度
        vb = back_car.get_speed()
        # 前车速度
        vf = front_car.get_speed()
        # 跟车距离
        # 下面是公式转化部分
        ss = 2 + vb * 1.2 + vb * (abs(vb - vf)) / (2 * pow(1.5 * 2, 0.5))
        k = 1
        if ss <= 0:
            k = 0
        acc = 1.5 * (1 - pow(vb / self.iv, 4) -k * pow(ss / (distance - front_car_length), 2))

        return acc

    def idm_change_left(self):
        waypoint = self.get_waypoint()
        if waypoint.get_left_lane() is None:
            print('已是最左车道，无法变道！')  # 待图形化展示
            return False

        road_id,lane_id = waypoint.road_id.waypoint.lane_id
        distance = self.get_waypoint_distance()
        distance_backup = distance

        # 求得同目标车辆列表
        followList = self.get_follow_list(road_id, lane_id, distance, Globe_Var.IDM_RANGE)
        lfollowList = self.get_follow_list(road_id, lane_id - 1, distance, Globe_Var.IDM_RANGE)

        # 按距离排序
        followList.sort(key=lambda x: x[1])
        lfollowList.sort(key=lambda x: x[1])

        FVC, FV, PFVC, PFV = 0, 0, 0, 0

        vehicle, distance = followList[0]
        FVC += self.idm_follow_model(distance, vehicle, self.vehicle)
        for i in range(1, len(followList)):
            vehicle, distance = followList[i]
            front_car, fdistance = followList[i - 1]
            FVC += self.idm_follow_model(distance - fdistance, vehicle,front_car)

        vehicle, distance = lfollowList[0]
        PFVC += self.idm_follow_model(distance, vehicle, self.vehicle)

        for i in range(1, len(lfollowList)):
            vehicle, distance = lfollowList[i]
            front_car, fdistance = lfollowList[i - 1]
            PFVC += self.idm_follow_model(distance - fdistance, vehicle,
                                          front_car)
        for i in range(len(followList)):
            vehicle, distance = followList[i]
            FV += self.get_accelerometer()
        for i in range(len(lfollowList)):
            vehicle, distance = lfollowList[i]
            PFV += self.get_accelerometer()

        asv = self.vehicle.get_acceleration()
        pasv = self.idm_follow_model(distance_backup, self.vehicle,self.get_forward_vehicle(road_id, lane_id, distance_backup))

        # 下面是公式转化部分
        usv = pasv - asv + 0.1 * (FVC - FV + PFVC - PFV)
        if pasv < -2 or PFVC < -2:
            self.idm_follow_model(self.get_waypoint_distance(followList[0][0]),followList[0][0], self.vehicle)
            print('向左变道失败！')
            return False

        if usv <= 0.3:
            self.idm_follow_model(self.get_waypoint_distance(followList[0][0]),followList[0][0], self.vehicle)
            print('向左变道失败！')
            return False

        self.lane_change = 'left'
        self.change_lane(self.lane_change)
        print('向左变道成功！')
        return True

    def idm_change_right(self):
        waypoint = self.get_waypoint(self.vehicle)
        if waypoint.get_right_lane() is None:
            print('已是最右车道，无法变道！')  # 待图形化展示
            return False

        road_id = waypoint.road_id
        lane_id = waypoint.lane_id
        distance = self.get_waypoint_distance()
        distance_backup = distance

        # 求得同目标车辆列表
        followList = self.get_follow_list(road_id, lane_id, distance, 300)
        lfollowList = self.get_follow_list(road_id, lane_id + 1, distance, 300)

        # 按距离排序
        followList.sort(key=lambda x: x[1])
        lfollowList.sort(key=lambda x: x[1])

        FVC, FV, PFVC, PFV = 0, 0, 0, 0

        vehicle, distance = followList[0]
        FVC += self.idm_follow_model(distance, vehicle, self.vehicle)
        for i in range(1, len(followList)):
            vehicle, distance = followList[i]
            front_car, fdistance = followList[i - 1]
            FVC += self.idm_follow_model(distance - fdistance, vehicle,front_car)

        vehicle, distance = lfollowList[0]
        PFVC += self.idm_follow_model(distance, vehicle, self.vehicle)

        for i in range(1, len(lfollowList)):
            vehicle, distance = lfollowList[i]
            front_car, fdistance = lfollowList[i - 1]
            PFVC += self.idm_follow_model(distance - fdistance, vehicle, front_car)

        for i in range(len(followList)):
            vehicle, distance = followList[i]
            FV += self.get_accelerometer()

        for i in range(len(lfollowList)):
            vehicle, distance = lfollowList[i]
            PFV += self.get_accelerometer()

        asv = self.vehicle.get_acceleration()
        pasv = self.idm_follow_model(distance_backup, self.vehicle,self.get_forward_vehicle(road_id, lane_id, distance_backup))

        # 下面是公式转化部分
        usv = pasv - asv + 0.1 * (FVC - FV + PFVC - PFV)
        if pasv < -2 or PFVC < -2:
            self.idm_follow_model(self.get_waypoint_distance(followList[0][0]),followList[0][0], self.vehicle)
            print('向右变道失败！')
            return False

        if usv <= 0.3:
            self.idm_follow_model(self.get_waypoint_distance(followList[0][0]),followList[0][0], self.vehicle)
            print('向右变道失败！')
            return False

        self.lane_change = 'right'
        self.change_lane(self.lane_change)
        print('向右变道成功！')

        return True
