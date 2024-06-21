#! /usr/bin/env python3
#coding=utf-8




import rospy, sys
import signal
import serial, time
from std_msgs.msg import String


def getch():
	try:
		ch = sys.stdin.read(1)
	except IOError:
		pass
	return ch

def sigint_handler(sig, frame):
    # 执行特定的中断处理操作
    # 例如，关闭打开的文件、释放资源等
    # 然后退出程序
    print("\nReceived SIGINT, exiting...")
    sys.exit(0)


if __name__ == "__main__":

    #1.初始化 ROS 节点

    rospy.init_node("turtle")

    #2.创建发布者对象

    pub = rospy.Publisher("turtle_serial",String, queue_size=1)
    #4.编写消息发布逻辑
    signal.signal(signal.SIGINT, sigint_handler) 
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # 获取键盘输入
        last_value = 5
        input_value = input()
        if input_value != '' and input_value.isdigit():
            input_value = int(input_value)
            if input_value in [0, 2, 4, 5, 6, 8]:
                # 发布输入值到话题
                rospy.loginfo("Publishing: %d", input_value)
                pub.publish(str(input_value))
                last_value = input_value
        else:
            pass
        rate.sleep()


