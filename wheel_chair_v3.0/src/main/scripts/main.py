#! /usr/bin/env python3
# coding=utf-8


import rospy
import sys
from std_msgs.msg import String
import signal
import threading
import core

lock = threading.Lock()
data = {1: 5, 2:5,3:5,4:5, 5:0} #
mode = 0 # 0 is no_app, 1 is app_handle, 2 is app_track, 3 is turtle


def dealData():
    # print(f"处理数据{data}")
	#with lock:
	if int(data[5]) != 0: 
		return data[5]
	if int(data[2]) != 5:
		return data[2]
	if mode == 0:
		return data[1]
	elif mode == 1:
		#print(data[3])
		return data[3]
	elif mode == 2:
		return data[4]


def sendData(var):
    #print(f'发送指令给duoji{var}')
    core.ctl_duoji(int(var))


def deal_imu_msg(msg):
    #with lock:
    global data
    #print(f'type:1, value:{msg.data}')
    data[1] = msg.data
    #print("imu Over")


def deal_laser_msg(msg):
    # rospy.loginfo("I heard:%s",msg.data)
    #with lock:
    global data
    #print(f'type:2.la, value:{msg.data}')
    data[2] = msg.data
    #print("laser Over")


def deal_app_msg(msg):
    # rospy.loginfo("I heard:%s",msg.data)
    global data
    global mode
    s = str(msg.data)
    #print("I receiver + " + s)
    if len(s) != 0:
    	mode = int(s[0])
    	#print(mode)
    	if mode == 1:
        	data[3] = str(s[1])
        	#print(data[3])


def deal_openmv_msg(msg):
    # rospy.loginfo("I heard:%s",msg.data)
    #with lock:
    global data
    #print(f'type:4, value:{msg.data}')
    data[4] = msg.data
    #print("openmv Over")

def deal_turtle_msg(msg):
    # rospy.loginfo("I heard:%s",msg.data)
    #with lock:
    global data
    #print(f'type:4, value:{msg.data}')
    data[5] = msg.data
    #print("openmv Over")

def heartbeat_callback(event):
    sendData(dealData())

def sigint_handler(sig, frame):
    #print("\nReceived SIGINT, exiting...")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    rospy.init_node("center")
    rate = rospy.Rate(10)
    #sub_openmv = rospy.Subscriber("openmv_serial", String, deal_openmv_msg, queue_size=1)
    sub_imu = rospy.Subscriber("imu_serial", String, deal_imu_msg, queue_size=1)
    sub_laser = rospy.Subscriber("laser_serial", String, deal_laser_msg, queue_size=1)
    sub_turtle = rospy.Subscriber("turtle_serial", String, deal_turtle_msg, queue_size=1)
    sub_app = rospy.Subscriber("android_serial", String, deal_app_msg, queue_size=1)
    #while not rospy.is_shutdown():
    heartbeat_timer = rospy.Timer(rospy.Duration(0.1), heartbeat_callback)
    #sendData(dealData())
    rospy.spin()
    

