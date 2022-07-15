#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32, Float32
import sys
from amr_control import minimal_modbus
import pandas as pd


def right_vel_callback(msg):
    global rightSpeed
    rightSpeed = msg.data


def left_vel_callback(msg):
    global leftSpeed
    leftSpeed = msg.data


def filter_encoders(right_encoder, left_encoder):
    global right_diff
    global left_diff
    global first_read
    global prev_right_encoder
    global prev_left_encoder
    global list_right_encode
    global list_left_encode

    size = 5
    if len(list_left_encode) < size and len(list_right_encode) < size:
        list_left_encode.append(left_encoder)
        list_right_encode.append(right_encoder)
    elif len(list_left_encode) >= size and len(list_right_encode) >= size:
        list_right_encode.pop(0)
        list_left_encode.pop(0)
        list_left_encode.append(left_encoder)
        list_right_encode.append(right_encoder)

    df = pd.DataFrame(
        {
            "left": list_left_encode,
            "right": list_right_encode
        }

    )
    desc = df.describe()
    left_Quartile = desc.left[4:7]
    right_Quartile = desc.right[4:7]
    IQR = {
        "left": left_Quartile[2]-left_Quartile[0],
        "right": right_Quartile[2]-right_Quartile[0]
    }

    K = 1.0
    Outlier_limits = {
        "left":
        {
            "lower": left_Quartile[0]-K*IQR["left"],
            "upper": left_Quartile[2]+K*IQR["left"]
        },
        "right":
        {
            "lower": right_Quartile[0]-K*IQR["right"],
            "upper": right_Quartile[2]+K*IQR["right"]
        }
    }

    if right_encoder >= Outlier_limits["right"]["lower"] and right_encoder <= Outlier_limits["right"]["upper"]:
        pass
    else:
        rospy.logerr("Right Error: " + str(right_encoder))
        raise Exception()

    if left_encoder >= Outlier_limits["left"]["lower"] and left_encoder <= Outlier_limits["left"]["upper"]:
        pass
    else:
        rospy.logerr("Left Error: " + str(left_encoder))
        raise Exception()


def publish_encoder():

    global right_diff
    global left_diff
    global first_read
    global prev_right_encoder
    global prev_left_encoder
    global list_right_encode
    global list_left_encode

    right_encoder = robot.read_encoder_absolute_status("right")
    left_encoder = robot.read_encoder_absolute_status("left")

    if first_read:
        prev_right_encoder = right_encoder
        prev_left_encoder = left_encoder
        first_read = False

    right_diff = right_encoder - prev_right_encoder
    left_diff = left_encoder - prev_left_encoder

    # filtreleme
    try:
        filter_encoders(right_encoder, left_encoder)
    except:
        return

    # rospy.loginfo("Right: " + str(right_diff) +
    #               " Left: " + str(left_diff))

    tick_per_degree = 583

    right_degree = right_diff / tick_per_degree
    left_degree = left_diff / tick_per_degree

    right_encoder_pub.publish(right_degree)
    left_encoder_pub.publish(left_degree)

    prev_right_encoder = right_encoder
    prev_left_encoder = left_encoder


if __name__ == "__main__":
    args = sys.argv
    if len(args) == 1:
        port = "/dev/ttyUSB0"
    else:
        port = args[1]

    rospy.init_node('motor_controller')
    robot = minimal_modbus.MotorDevice(port, 115200)
    rightSpeed = 0
    leftSpeed = 0

    right_diff = 0
    left_diff = 0

    first_read = True

    prev_right_encoder = None
    prev_left_encoder = None

    list_left_encode = []
    list_right_encode = []

    right_encoder_pub = rospy.Publisher('right_ticks', Float32, queue_size=100)
    left_encoder_pub = rospy.Publisher('left_ticks', Float32, queue_size=100)

    rospy.Subscriber('right_vel', Int32, right_vel_callback)
    rospy.Subscriber('left_vel', Int32, left_vel_callback)

    rate = rospy.Rate(100)
    print("started motor_controller node")
    maxRPM = 1500
    while not rospy.is_shutdown():
        if rospy.get_param('/emercencyStop', False):
            robot.write_speed_command("right", 0)
            robot.write_speed_command("left", 0)
            rospy.loginfo("Stopping")
        elif rightSpeed > maxRPM or leftSpeed > maxRPM:
            robot.write_speed_command("right", maxRPM)
            robot.write_speed_command("left", maxRPM)
            rospy.loginfo("Max Speed")
        else:
            robot.write_speed_command("right", rightSpeed)
            robot.write_speed_command("left", leftSpeed)
        publish_encoder()
        rate.sleep()
