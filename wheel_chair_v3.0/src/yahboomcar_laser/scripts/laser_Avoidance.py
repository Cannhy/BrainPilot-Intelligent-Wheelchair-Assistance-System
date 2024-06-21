#!/usr/bin/env python3
# coding:utf-8
import math
import numpy as np
import time
import rospy
from std_msgs.msg import String
from common import *
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server

# from yahboomcar_laser.cfg import laserAvoidPIDConfig

RAD2DEG = 180 / math.pi


class laserAvoid:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(10)  # 频率设置
        # self.Moving = False
        # self.switch = False
        self.ros_ctrl = ROSCtrl()
        # Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)
        # self.linear = 0.5
        # self.angular = 1.0
        self.ResponseDist = 0.6  # 反应距离
        self.Front_warning = 0
        # self.LaserAngle = 45
        self.blindDist = 0.2  # 盲区距离、轮椅本身干扰
        self.ResponseAmount = 15  # 预警阈值
        self.chairWidthL = 0.6  # 左侧轮椅宽度
        self.chairWidthR = 0.05  # 右侧轮椅宽度
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        print("laser_avoidance node start")

    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down laser_avoidance node.")

    # def dynamic_reconfigure_callback(self, config, level):
    #     self.switch = config['switch']
    #     self.linear = config['linear']
    #     self.angular = config['angular']
    #     self.LaserAngle = config['LaserAngle']
    #     self.ResponseDist = config['ResponseDist']
    #     return config

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan):
            print("information type error")
            return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        # Record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(scan_data.ranges)
        # self.Right_warning = 0
        # self.Left_warning = 0
        self.Front_warning = 0
        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        # if we already have a last scan to compare to
        minDistList = []
        minDistIDList = []
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            front_distance = ranges[i] * math.cos((180 - abs(angle)) / 180 * math.pi)
            side_distance = ranges[i] * math.sin((180 - abs(angle)) / 180 * math.pi)  # 绝对距离，不区分左右
            if angle > 0:
                side_distance = side_distance - self.chairWidthL  # 减去左侧轮椅宽度，若最终值小于0则有碰撞风险（右扶手左侧）
            else:
                side_distance = side_distance - self.chairWidthR  # 减去右侧侧轮椅宽度，若最终值小于0则有碰撞风险（右扶手右侧）
            # if ranges[i] <= self.ResponseDist and ranges[i] > self.blindDist:
            if self.ResponseDist >= front_distance > self.blindDist and side_distance < 0:
                record = False
                # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
                # 通过清除不需要的扇区的数据来保留有效的数据
                # 以下角度有待调整
                # if 160 > angle > 120:
                #     self.Right_warning += 1
                #     record = True
                # elif -160 < angle < -120:
                #     self.Left_warning += 1
                #     record = True
                # TODO:狭窄巷子中，轮椅转弯，会发生什么？或许避障命令应该只是“停止前进”
                if 150 < angle or angle < -100:
                    self.Front_warning += 1
                    record = True
                if record:
                    minDistList.append(ranges[i])
                    minDistIDList.append(angle)
        if len(minDistIDList) != 0:
            minDist = min(minDistList)
            angle = minDistIDList[minDistList.index(minDist)]
            front_distance = minDist * math.cos((180 - abs(angle)) / 180 * math.pi)
            side_distance = minDist * math.sin((180 - abs(angle)) / 180 * math.pi)
            if angle > 0:
                side_distance = side_distance - self.chairWidthL
            else:
                side_distance = side_distance - self.chairWidthR
            #print("minDis:" + str(minDist) + " angle:" + str(angle) + " frontDis:" + str(
                #front_distance) + " sideDis:" + str(side_distance))
        #print("Front_warning:" + str(self.Front_warning))
        if self.Front_warning > self.ResponseAmount:
            pub.publish("8")
            print("stop go front")  # 应替换为运动指令
        else:
            pub.publish("5")
        self.r.sleep()
        # else : self.ros_ctrl.pub_vel.publish(Twist())


if __name__ == '__main__':
    rospy.init_node('laser_Avoidance', anonymous=False)
    pub=rospy.Publisher('laser_serial',String,queue_size=1)
    tracker = laserAvoid()
    rospy.spin()

