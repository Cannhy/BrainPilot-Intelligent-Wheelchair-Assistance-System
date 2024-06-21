#! /usr/bin/env python3
# coding=utf-8
# 添加uservo.py的系统路径
import sys

sys.path.append("../../src")
# 导入依赖
import time
import struct
import serial
from uservo import UartServoManager

# 参数配置
# 角度定义
SERVO_PORT_NAME = '/dev/ttyUSB0'  # 舵机串口号
SERVO_BAUDRATE = 57600  # 舵机的波特率
SERVO_ID0 = 0  # 舵机的ID号
SERVO_ID1 = 1
SERVO_HAS_MTURN_FUNC = False  # 舵机是否拥有多圈模式

# 初始化串口
uart0 = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE, \
                      parity=serial.PARITY_NONE, stopbits=1, \
                      bytesize=8, timeout=0)
# uart1 = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,\
# 					 parity=serial.PARITY_NONE, stopbits=1,\
# 					 bytesize=8,timeout=0)
# 初始化舵机管理器
uservo0 = UartServoManager(uart0, is_debug=True)
# uservo0 = UartServoManager(uart0, is_debug=True)
# uservo0.wait() # 等待舵机静止
# uservo0.wait() # 等待舵机静止


uservo0.set_servo_angle(SERVO_ID0, 0.0, interval=1000)
uservo0.wait()  # 等待舵机静止
uservo0.set_servo_angle(SERVO_ID1, 0.0, interval=1000)
uservo0.wait()  # 等待舵机静止

print("RESETTED")

last_state = 0


def angle_reset():
    uservo0.set_servo_angle(SERVO_ID0, 0.0, interval=1000)
    uservo0.wait()  # 等待舵机静止
#    uservo0.set_servo_angle(SERVO_ID1, 0.0, interval=1000)
#    uservo0.wait()  # 等待舵机静止


def ctl_duoji(ctlNum):
    global last_state
    if ctlNum == 2:  # 前
        #print("F 0->92,1->0")
        if last_state != 2:
            angle_reset()
            uservo0.set_servo_angle(SERVO_ID1, 0.0, interval=600)
            uservo0.wait()
            uservo0.set_servo_angle(SERVO_ID0, 110.0, interval=600)
            uservo0.wait()  # 等待舵机静止
            last_state = 2
    elif ctlNum == 4:  # 左
        #print("L 0->92,1->90")
        if last_state != 4:
            angle_reset()
            uservo0.set_servo_angle(SERVO_ID1, 100.0, interval=600)
            uservo0.wait()  # 等待舵机静止
            uservo0.set_servo_angle(SERVO_ID0, 120.0, interval=600)
            uservo0.wait()  # 等待舵机静止
            last_state = 4
    elif ctlNum == 5:  # 停
        #print("S 0->0,1->0")
        if last_state != 5:
            angle_reset()
            uservo0.set_servo_angle(SERVO_ID1, 0.0, interval=600)
            uservo0.wait()
            uservo0.set_servo_angle(SERVO_ID0, 0.0, interval=600)
            uservo0.wait()
            last_state = 5
    elif ctlNum == 6:  # 右
        #print("R 0->92,1->-90")
        if last_state != 6:
            angle_reset()
            uservo0.set_servo_angle(SERVO_ID1, -90.0, interval=600)
            uservo0.wait()  # 等待舵机静止
            uservo0.set_servo_angle(SERVO_ID0, 110.0, interval=600)
            uservo0.wait()  # 等待舵机静止
            last_state = 6
    elif ctlNum == 8:  # 后
        #print("B 0->92,1->180")
        if last_state != 8:
            angle_reset()
            uservo0.set_servo_angle(SERVO_ID1, 180.0, interval=600)
            uservo0.wait()  # 等待舵机静止
            uservo0.set_servo_angle(SERVO_ID0, 110.0, interval=600)
            uservo0.wait()  # 等待舵机静止
            last_state = 8

# x = 3
# while True:
#     x = x + 1
#     x = x%9 + 1
#     for a in range(10):
#         ctl_duoji(x)

