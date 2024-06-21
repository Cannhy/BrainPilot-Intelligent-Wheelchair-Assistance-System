# AprilTags Example
#
# This example shows the power of the OpenMV Cam to detect April Tags
# on the OpenMV Cam M7. The M4 versions cannot detect April Tags.

import sensor, image, time, math
from machine import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # we run out of memory if the resolution is much bigger...
sensor.skip_frames(30)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
sensor.set_auto_exposure(False,exposure_us=50000)

clock = time.clock()
uart = UART(3, 115200)

# 注意！与find_qrcodes不同，find_apriltags 不需要软件矫正畸变就可以工作。

# 注意，输出的姿态的单位是弧度，可以转换成角度，但是位置的单位是和你的大小有关，需要等比例换算

# f_x 是x的像素为单位的焦距。对于标准的OpenMV，应该等于2.8/3.984*656，这个值是用毫米为单位的焦距除以x方向的感光元件的长度，乘以x方向的感光元件的像素（OV7725）
# f_y 是y的像素为单位的焦距。对于标准的OpenMV，应该等于2.8/2.952*488，这个值是用毫米为单位的焦距除以y方向的感光元件的长度，乘以y方向的感光元件的像素（OV7725）

# c_x 是图像的x中心位置
# c_y 是图像的y中心位置

f_x = (2.8 / 3.984) * 160  # 默认值
f_y = (2.8 / 2.952) * 120  # 默认值
c_x = 160 * 0.5  # 默认值(image.w * 0.5)
c_y = 120 * 0.5  # 默认值(image.h * 0.5)
k = 5.483  # origin_x/k= actual_x (m)
direction = 0  # -1为左,+1为右
moving = 0  # -1为后,+1为前
start_d = 0.3
end_d = 0.1
start_f = 1.0
start_b = 0.5
end_m = 0.7
command = 0
no_find_num = 0


def degrees(radians):
    return (180 * radians) / math.pi


while (True):
    clock.tick()
    time.sleep(0.05)
    img = sensor.snapshot()
    tags = img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y)
    if len(tags) == 0:
        print("no tag find!")
        no_find_num += 1
        if no_find_num > 8:
            command = 5
            direction = 0
            moving = 0
            no_find_num = 0
            print("stop because no tag find!")
    elif len(tags) >= 2:
        command = 5
        direction = 0
        moving = 0
        no_find_num = 0
        print("not the only tag!")
    else:
        # for tag in tags: # 默认为TAG36H11
        no_find_num = 0
        tag = tags[0]
        img.draw_rectangle(tag.rect(), color=(255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))
        print_args = (tag.x_translation(), tag.y_translation(), tag.z_translation(), \
                      degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))
        # 位置的单位是未知的，旋转的单位是角度
        actual_x = tag.x_translation() / k
        actual_y = tag.y_translation() / k
        actual_z = tag.z_translation() / k
        angle_x = degrees(tag.x_rotation())
        angle_y = degrees(tag.y_rotation())
        angle_z = degrees(tag.z_rotation())
        if abs(actual_x) > start_d:
            if actual_x > start_d:
                direction = 1
            else:
                direction = -1
        elif direction != 0 and abs(actual_x) < end_d:
            direciton = 0
        if abs(actual_z) > start_f:
            moving = 1
        elif abs(actual_z) < start_b:
            moving = -1
        elif moving == 1 and abs(actual_z) < end_m:
            moving = 0
        elif moving == -1 and abs(actual_z) > end_m:
            moving = 0
        if direction == 0 and moving == 0:
            command = 5
        elif direction != 0:
            if direction == -1:
                command = 4
            else:
                command = 6
        elif moving != 0:
            if moving == 1:
                command = 2
            else:
                command = 8
        print_args = (actual_x, actual_y, actual_z, angle_x, angle_y, angle_z)
        print("Tx: %f, Ty %f, Tz %f, Rx %f, Ry %f, Rz %f" % print_args)
    # print("direction: %d ,moving: %d ,command: %d" % (direction, moving, command))
    print(str(command))
    print(clock.fps())
