#!/usr/bin/env python3

from numpy import average
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import  Float32
from geometry_msgs.msg import Twist


def front_cb(data):
    # if not rospy.get_param("/correct_orientation", False):
    #     return
    global first_perfect_yaw
    global stack
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    ranges = [round(i, 2) for i in ranges]
    # print("Front " + str(len(ranges)))
    # print(angle_min*180/3.14)
    # print(angle_max*180/3.14)
    # print(angle_increment*180/3.14)
    # front_right = min(ranges[0:90])
    front = min(ranges[728:728+10])
    if front <= 0.5 and first_perfect_yaw:
        rospy.set_param("/correct_orientation", False)
        cmd_vel.publish(Twist())
    # front_left = min(ranges[270:360])
    most_right_value = average(ranges[0:10])
    most_left_value = average(ranges[1466:1466+10])
    print("front most right value:" + str(most_right_value))
    print("front most left value:" + str(most_left_value))
    # print("left right difference:" + str(abs(most_right_value-most_left_value)))
    # msg = Twist()
    # msg.linear.x = 0

    # if most_right_value < 0.69:  # and most_left_value > 0.25:
    #     print("robot needs to tilt left")
    #     msg.angular.z = 0.05
    #     stack = []
    # elif most_right_value > 0.71:  # and most_left_value < 0.22:
    #     print("robot needs to tilt right")
    #     msg.angular.z = -0.05
    #     stack = []

    # elif most_right_value >= 0.69 and most_right_value <= 0.71 and most_left_value <= 0.25 and most_left_value >= 0.22:
    #     print("robot orientation is perfect")
    #     msg.angular.z = 0
    #     stack.append("perfect")
    # cmd_vel.publish(msg)

    # print(stack)
    # if len(stack) >= 10:
    #     print("EXIT")
    #     rospy.set_param("/correct_orientation", False)
    #     stack = []


def imu_cb(data):
    global stack
    global first_perfect_yaw
    if not rospy.get_param("/correct_orientation", False):
        return
    yaw_degree = data.data
    docking_degree = 0
    # print(yaw_degree)
    msg = Twist()
    msg.linear.x = 0

    if yaw_degree < docking_degree - 0.1:
        print("robot needs to tilt left")
        msg.angular.z = max(yaw_degree / 100, 0.1)
        stack = []
    elif yaw_degree > docking_degree + 0.1:
        print("robot needs to tilt right")
        msg.angular.z = - max(yaw_degree / 100, 0.1)
        stack = []

    elif yaw_degree >= docking_degree-0.1 and yaw_degree <= docking_degree+0.1:
        print("robot orientation is perfect")
        msg.angular.z = 0
        stack.append("perfect")

    if first_perfect_yaw:
        msg.linear.x = 0.5
    cmd_vel.publish(msg)

    if len(stack) >= 10:
        first_perfect_yaw = True
        print("EXIT")
        # rospy.set_param("/correct_orientation", False)
        stack = []


def back_cb(data):
    # if not rospy.get_param("/correct_orientation", False):
    #     return
    global first_perfect_yaw
    global stack
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    ranges = [round(i, 2) for i in ranges]
    # print("Front " + str(len(ranges)))
    # print(angle_min*180/3.14)
    # print(angle_max*180/3.14)
    # print(angle_increment*180/3.14)
    # front_right = min(ranges[0:90])
    front = min(ranges[728:728+10])
    # front_left = min(ranges[270:360])
    most_right_value = average(ranges[0:10])
    most_left_value = average(ranges[1466:1466+10])
    print("back most right value:" + str(most_right_value))
    print("back most left value:" + str(most_left_value))
    # print("left right difference:" + str(abs(most_right_value-most_left_value)))
    # msg = Twist()
    # msg.linear.x = 0

    # if most_right_value < 0.69:  # and most_left_value > 0.25:
    #     print("robot needs to tilt left")
    #     msg.angular.z = 0.05
    #     stack = []
    # elif most_right_value > 0.71:  # and most_left_value < 0.22:
    #     print("robot needs to tilt right")
    #     msg.angular.z = -0.05
    #     stack = []

    # elif most_right_value >= 0.69 and most_right_value <= 0.71 and most_left_value <= 0.25 and most_left_value >= 0.22:
    #     print("robot orientation is perfect")
    #     msg.angular.z = 0
    #     stack.append("perfect")
    # cmd_vel.publish(msg)

    # print(stack)
    # if len(stack) >= 10:
    #     print("EXIT")
    #     rospy.set_param("/correct_orientation", False)
    #     stack = []


if __name__ == "__main__":
    stack = []
    first_perfect_yaw = False
    rospy.init_node('docking')
    sub_front = rospy.Subscriber("/f_scan_rep117", LaserScan, front_cb)
    sub_back = rospy.Subscriber("/b_scan_rep117", LaserScan, back_cb)
    sub_imu = rospy.Subscriber("/yaw_degree", Float32, imu_cb)

    # front_pub = rospy.Publisher("/front_safety", String, queue_size=10)
    # back_pub = rospy.Publisher("/back_safety", String, queue_size=10)

    cmd_vel = rospy.Publisher("/cmd_vel_unfiltered", Twist, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
