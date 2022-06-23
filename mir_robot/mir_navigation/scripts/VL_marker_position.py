#!/usr/bin/env python3

from cmath import inf

from cv2 import mean
import rospy
import rospkg
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time

def is_close(one,two):
    if abs(one-two) < 0.05:
        return True
    else:
        return False
    
def front_cb(data):
    global front_state
    global start_time
    front_state = "NONE"
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    ranges = [round(i, 2) for i in ranges]
    ranges = [i if i < 5 else inf for i in ranges]

    # print(len(ranges)) # 541
    # # print()
    # left = ranges[0:270]
    # right =  ranges[270:]
    # print(right)
    # print()
    # # print(right)
    # front_right = min(ranges[0:90])
    # front = min(ranges[90:270])
    # front_left = min(ranges[270:360])
    left_front = ranges[175:180]
    middle = ranges[180]
    right_front = ranges[180:185]

    left_front_avg = round(sum(left_front)/len(left_front),2)
    right_front_avg = round(sum(right_front)/len(right_front),2)

    # print("left avg: " + str(left_front_avg))
    # print("Middle" + str(middle))
    # print("right avg: " + str(right_front_avg))

    # start_time = None
    if(is_close(left_front_avg,right_front_avg) and right_front_avg<middle):
        print("At V")
        if start_time is None:
            start_time = time.time()
        else:
            elapsed_time = time.time() - start_time
            if elapsed_time > 5:
                start_time = None
                print("5 sec has passed")

    # print("****")
    # print(ranges[180:200])
    # for angle in np.arange(angle_min, angle_max, angle_increment):
    #     if angle == 0:
    #         print("Reading at angle: " + str(angle*180/math.pi) + " is: ["+str(i)+"] " + str(ranges[i]))

    # i = 0
    # for angle in np.arange(angle_min, angle_max, angle_increment):
    #     if ranges[i] == inf:
    #         pass
    #     elif angle*180/math.pi > 0 and angle*180/math.pi < 90:
    #         print("Reading at angle: " + str(angle*180/math.pi) + " is: ["+str(i)+"] " + str(ranges[i]))
    #     i += 1
    # print("*****************************")


if __name__ == "__main__":
    front_state = "NONE"
    back_state = "NONE"
    safety = True
    start_time = None
    rospy.init_node('VL')
    sub_front = rospy.Subscriber("/f_scan_rep117", LaserScan, front_cb)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
