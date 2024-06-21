#! /usr/bin/env python3
# coding=utf-8

import os
import sys
import signal
import rospy
from std_msgs.msg import String


def deal_app_msg(msg):
    #print("rece m:" + str(msg.data))
    pub.publish(str(msg.data))

def sigint_handler(sig, frame):
    print("\nReceived SIGINT, exiting...")
    sys.exit(0)

# def main(getData):
if __name__ == "__main__":
	signal.signal(signal.SIGINT, sigint_handler)
	rospy.init_node("android")
	rate = rospy.Rate(10)
	pub = rospy.Publisher("android_serial", String, queue_size=1)
	sub_imu = rospy.Subscriber("/remote", String, deal_app_msg, queue_size=1)
	rospy.spin()

