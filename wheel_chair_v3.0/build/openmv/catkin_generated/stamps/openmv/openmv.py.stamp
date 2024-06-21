#! /usr/bin/env python
#coding=utf-8

#! /usr/bin/env python


import rospy, sys
import signal
import serial, time
from std_msgs.msg import String

def sigint_handler(sig, frame):
    # 执行特定的中断处理操作
    # 例如，关闭打开的文件、释放资源等
    # 然后退出程序
    print("\nReceived SIGINT, exiting...")
    sys.exit(0)


if __name__ == "__main__":

    #1.初始化 ROS 节点

    rospy.init_node("openmv")

    #2.创建发布者对象

    pub = rospy.Publisher("openmv_serial",String, queue_size=1)

    print("openmv串口通信开始")
    Port = "/dev/ttyACM0"  # 串口
    baudRate = 115200  # 波特率
    ser = serial.Serial(Port, baudRate, timeout=1)
    #4.编写消息发布逻辑
    signal.signal(signal.SIGINT, sigint_handler) 
    rate = rospy.Rate(25)
    num = 1
    strs=""
    while not rospy.is_shutdown():
        strs = ser.readline().strip().decode()
        if strs != "":
            pub.publish(strs)  #发布消息
            #print(strs)
        num += 1
        rate.sleep()  #休眠


