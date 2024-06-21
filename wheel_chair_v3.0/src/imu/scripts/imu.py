#! /usr/bin/env python3
# coding=utf-8

import os
import sys
path = os.path.abspath(".")
sys.path.insert(0, path+"src/pkg/scripts")
import bleak
import device_model
from device_model import DeviceModel
import asyncio
import signal
import parseData
import rospy
from std_msgs.msg import String
# 扫描到的设备 Scanned devices
devices = []


# 扫描蓝牙设备并过滤名称 Scan Bluetooth devices and filter names
async def scan():
    global devices
    find = []
    print("Searching for Bluetooth devices......")
    try:
        for x in range(2):
            devices += await bleak.BleakScanner.discover()
        print("Search ended")
        for d in devices:
            if d.name is not None and "WT" in d.name:
                find.append(d)
                print(d)
        if len(find) == 0:
            print("No devices found in this search!")
    except Exception as ex:
        print("Bluetooth search failed to start")
        print(ex)

getDataFunc = None


def parseImuData(data):
    print("处理imu数据")


buffer = []
# 数据更新时会调用此方法 This method will be called when data is updated
def updateData(DeviceModel):
    # 直接打印出设备数据字典 Directly print out the device data dictionary
    #print(DeviceModel.deviceData)
    # 获得X轴加速度 Obtain X-axis acceleration
    op = parseData.process_ang(DeviceModel.deviceData)    # 处理imu数据
    #if len(buffer) == 0:
        #buffer.append(op)
    #print(numa)
    #print(numa)
    pub.publish(str(op))
    #print(numa)
    #print(numa)
    #else:
        #buffer[0] = op


def sigint_handler(sig, frame):
    print("\nReceived SIGINT, exiting...")
    sys.exit(0)

# def main(getData):
if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    rospy.init_node("imu")
    numa = "1231123123123123123"
    pub = rospy.Publisher("imu_serial", String, queue_size=1)
    #rate = rospy.Rate(1)
    # 搜索设备 Search Device
    asyncio.run(scan())
    # 选择要连接的设备 Select the device to connect to
    device_mac = "DC:A0:E3:2E:03:CF"
    user_input = "DC:A0:E3:2E:03:CF"
    # user_input = input("Please enter the Mac address you want to connect to (e.g. DF:E9:1F:2C:BD:59)：")
    for device in devices:
        if device.address == user_input:
            device_mac = device.address
            break
    if device_mac is not None:
        # 创建设备 Create device
        device = device_model.DeviceModel("MyBle5.0", device_mac, updateData)
        asyncio.run(device.openDevice())
        #while not rospy.is_shutdown():
            #if len(buffer) != 0:
                #pub.publish(buffer[0])
            #rate.sleep()
    else:
        print("No Bluetooth device corresponding to Mac address found!!")



