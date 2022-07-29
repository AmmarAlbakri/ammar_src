#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32, Float32
import sys
from amr_control import minimal_modbus
import pandas as pd
from amr_services.srv import Lift, LiftResponse


def filter_encoders(encoder):
    global first_read
    global prev_encoder

    size = 5
    if len(list_encode) < size:
        list_encode.append(encoder)
    elif len(list_encode) >= size:
        list_encode.pop(0)
        list_encode.append(encoder)

    df = pd.DataFrame(
        {
            "lift": list_encode,
        }

    )
    desc = df.describe()
    Quartile = desc.lift[4:7]
    IQR = {
        "lift": Quartile[2]-Quartile[0],
    }

    K = 1.0
    Outlier_limits = {
        "lift":
        {
            "lower": Quartile[0]-K*IQR["lift"],
            "upper": Quartile[2]+K*IQR["lift"]
        }

    }

    if encoder >= Outlier_limits["right"]["lower"] and encoder <= Outlier_limits["right"]["upper"]:
        pass
    else:
        rospy.logerr("Right Error: " + str(encoder))
        raise Exception()


def calculate_encoder():
    global diff
    global first_read
    global prev_encoder
    global total_degree
    global speed
    global encoder_error

    encoder = robot.read_encoder_absolute_status()

    if first_read:
        prev_encoder = encoder
        first_read = False

    diff = encoder - prev_encoder

    tolerance = abs(speed) * 6
    if encoder_error:
        tolerance = tolerance * 2
        encoder_error = False

    if abs(diff) > tolerance:
        rospy.logerr("encoder error" + str(diff))
        encoder_error = True
        # return
    else:
        # print(diff)
        prev_encoder = encoder
    # print("Encoder: " + str(encoder))
    # filtreleme
    # try:
    #     filter_encoders(encoder)
    # except:
    #     return

    tick_per_degree = 583
    total_degree = total_degree + (diff / tick_per_degree)
    print("Total Degrees:" + str(total_degree))


def run_lift(action):
    global speed
    global total_degree
    max_speed = 1000
    if action == "UP":
        speed = max_speed
    elif action == "DOWN":
        speed = -max_speed

    start_degree = total_degree
    while abs(start_degree - total_degree) < 1000:
        if speed == 0:
            break
        robot.write_speed_command(speed)
        calculate_encoder()

    robot.write_speed_command(0)
    print("Finish")


def lift(req):
    global speed
    print("Req:")
    print(req)
    if req.action == 'UP':
        run_lift("UP")
        return LiftResponse(1)
    elif req.action == 'DOWN':
        run_lift("DOWN")
        return LiftResponse(1)
    elif req.action == "STOP":
        speed = 0
        # robot.write_speed_command(0)
        return LiftResponse(1)
    else:
        print("Lift ERROR")
        return LiftResponse(-1)


if __name__ == "__main__":
    args = sys.argv
    if len(args) == 1:
        port = "/dev/ttyUSB1"
    else:
        port = args[1]

    rospy.init_node('lift_controller')
    robot = minimal_modbus.LiftDevice(port, 57600)
    s = rospy.Service('Lift', Lift, lift)

    speed = 0
    encoder_error = False
    diff = 0
    first_read = True
    total_degree = 0
    prev_encoder = None

    rate = rospy.Rate(60)
    print("started lift_controller node")

    while not rospy.is_shutdown():
        rate.sleep()

    robot.write_speed_command(0)
