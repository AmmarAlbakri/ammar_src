#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def front_cb(data):
    global front_state
    front_state = "NONE"
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    ranges = [round(i, 2) for i in ranges]
    # print("Front " + str(len(ranges)))
    front_right = min(ranges[0:90])
    front = min(ranges[90:270])
    front_left = min(ranges[270:360])

    # print("front_right " + str(front_right))
    # print("---")
    # print("front "  + str(front))
    # print("---")
    # print("front_left " + str(front_left))
    # print("---")

    if front <= 0.5:
        front_state = "DANGER"
    elif front > 0.5 and front <= 1.5:
        front_state = "WARNING"
    # print("Front state = " + front_state)
    rospy.set_param("/front_state",front_state)
    # global front_pub
    # front_pub.publish(front_state)


def back_cb(data):
    global back_state
    back_state = "NONE"
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    ranges = [round(i, 2) for i in ranges]
    # print("back " + str(len(ranges)))
    back_right = min(ranges[0:90])
    back = min(ranges[90:270])
    back_left = min(ranges[270:360])

    # print("back_right " + str(back_right))
    # print("---")
    # print("back "  + str(back))
    # print("---")
    # print("back_left " + str(back_left))
    # print("---")

    if back <= 0.5:
        back_state = "DANGER"
    elif back > 0.5 and back <= 1.5:
        back_state = "WARNING"

    # print("Back state = " + back_state)
    rospy.set_param("/back_state",back_state)
    # global back_pub
    # back_pub.publish(back_state)


if __name__ == "__main__":
    front_state = "NONE"
    back_state = "NONE"
    safety = True

    rospy.init_node('check_laser')
    sub_front = rospy.Subscriber("/f_scan_rep117", LaserScan, front_cb)
    sub_back = rospy.Subscriber("/b_scan_rep117", LaserScan, back_cb)

    # front_pub = rospy.Publisher("/front_safety", String, queue_size=10)
    # back_pub = rospy.Publisher("/back_safety", String, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
