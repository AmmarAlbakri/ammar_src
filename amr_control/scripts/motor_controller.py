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

    K = 1.2
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
    global rightSpeed
    global leftSpeed
    global encoder_error
    global number_of_encoder_error

    right_encoder = robot.read_encoder_absolute_status("right")
    left_encoder = robot.read_encoder_absolute_status("left")

    if first_read:
        prev_right_encoder = right_encoder
        prev_left_encoder = left_encoder
        first_read = False

    right_diff = right_encoder - prev_right_encoder
    left_diff = left_encoder - prev_left_encoder

    real_right_speed = robot.read_speed_status("right")
    real_left_speed = robot.read_speed_status("left")

    if real_right_speed > 30000:
        real_right_speed = 65536 - real_right_speed
    if real_left_speed > 30000:
        real_left_speed = 65536 - real_left_speed

    real_right_speed = max(real_right_speed, rightSpeed)
    real_left_speed = max(real_left_speed, leftSpeed)

    right_tolerance = abs(real_right_speed) * 22
    left_tolerance = abs(real_left_speed) * 22


    if encoder_error:
        right_tolerance = right_tolerance * 3
        left_tolerance = left_tolerance * 3
        encoder_error = False

    if number_of_encoder_error >= 3:
        prev_right_encoder = right_encoder
        prev_left_encoder = left_encoder

    # print("Left Speed: " + str(real_left_speed))
    # print("Right Speed: " + str(real_right_speed))

    if abs(left_diff) > left_tolerance and leftSpeed != 0:
        rospy.logerr("Left Encoder Error:" + str(left_diff))
        encoder_error = True
        number_of_encoder_error += 1
        return

    if abs(right_diff) > right_tolerance and rightSpeed != 0:
        rospy.logerr("Right Encoder Error:" + str(right_diff))
        encoder_error = True
        number_of_encoder_error += 1
        return

    # print("left diff: " + str(left_diff))
    # print("right diff: " + str(right_diff))

    tick_per_degree = 583

    right_degree = right_diff / tick_per_degree
    left_degree = left_diff / tick_per_degree

    right_encoder_pub.publish(right_degree)
    left_encoder_pub.publish(left_degree)
    prev_right_encoder = right_encoder
    prev_left_encoder = left_encoder
    number_of_encoder_error = 0


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

    encoder_error = False
    prev_right_encoder = None
    prev_left_encoder = None

    list_left_encode = []
    list_right_encode = []
    number_of_encoder_error = 0

    right_encoder_pub = rospy.Publisher('right_ticks', Float32, queue_size=100)
    left_encoder_pub = rospy.Publisher('left_ticks', Float32, queue_size=100)

    rospy.Subscriber('right_vel', Int32, right_vel_callback)
    rospy.Subscriber('left_vel', Int32, left_vel_callback)

    rate = rospy.Rate(100)
    print("started motor_controller node")
    maxRPM = 3000
    while not rospy.is_shutdown():
        if rospy.get_param('/emergencyStop', False):
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
        # print("Right Speed:" + str(rightSpeed) + " RPM")
        # print("Left Speed:" + str(robot.read_speed_status("left")) + " RPM")
        publish_encoder()
        rate.sleep()
    robot.write_speed_command("right", 0)
    robot.write_speed_command("left", 0)
