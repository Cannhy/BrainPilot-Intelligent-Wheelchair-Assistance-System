#! /usr/bin/env python3
# coding=utf-8
basic_x = 0
basic_y = 0
diff_f = 25
diff_s = 20
standard_count = -1
standard_setting = True
new_x = 0
new_y = 0

# 是否需加入连续检测？连续保持一角度时再发出命令？
# 是否加入斜率检测？角度变化到达一定速度时直接下达命令？
# |1|2|3|
# |4|5|6|
# |7|8|9|
def process_ang(data):
    global new_x
    global new_y
    global basic_x
    global basic_y
    global standard_count
    global standard_setting
    if standard_setting:
        new_x = 0
        new_y = 0
        standard_count = 5
        standard_setting = False
        print("start standard_setting")
        return 5
    if standard_count >= 1:
        if abs(data['AngX']) > 20 or abs(data['AngY']) > 20:
            return 5
        new_x += data['AngX'] / 5
        new_y += data['AngY'] / 5
        standard_count -= 1
        return 5
    elif standard_count == 0:
        basic_x = new_x
        basic_y = new_y
        standard_count -= 1
        print("standard_setting over: x: " + str(basic_x) + " y: " + str(basic_y))
        return 5
    real_x = data['AngX'] - basic_x
    real_y = data['AngY'] - basic_y
    '''
    if real_x <= -diff and real_y >= diff:
	output = 1
    elif real_x >= diff and real_y >= diff:
	output = 3
    elif real_x <= -diff and real_y <= -diff:
	output = 7
    elif real_x >= diff and real_y <= -diff:
	output = 9
	'''
    if real_x <= -diff_s:
        output = 4
    elif real_x >= diff_s:
        output = 6
    elif real_y >= diff_f:
        output = 2
    elif real_y <= -diff_f:
        output = 8
    else:
        output = 5
    return output


def start_standard_setting():
    global standard_setting
    standard_setting = True

# if __name__ == '__main__':
#     print(process_ang({'AngX': 0, 'AngY': 0, 'AngZ': 0}))
#     print(process_ang({'AngX': 20.0, 'AngY': 0, 'AngZ': 0}))
#     print(process_ang({'AngX': 0, 'AngY': 20.0, 'AngZ': 0}))
#     print(process_ang({'AngX': -20.0, 'AngY': 0, 'AngZ': 0}))
#     print(process_ang({'AngX': 0, 'AngY': -20.0, 'AngZ': 0}))
#     print(process_ang({'AngX': 20, 'AngY': -20.0, 'AngZ': 0}))
#     print(process_ang({'AngX': -20.3, 'AngY': 20.4, 'AngZ': 0}))
#     print(process_ang({'AngX': 20.0, 'AngY': 20.0, 'AngZ': 0}))
#     print(process_ang({'AngX': -20.0, 'AngY': -20.0, 'AngZ': 0}))

