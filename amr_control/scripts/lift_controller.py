#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32, Float32
import sys
from amr_control import minimal_modbus
import pandas as pd

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


def publish_encoder():
    global diff
    global first_read
    global prev_encoder
    global total_degree

    encoder = robot.read_encoder_absolute_status()

    if first_read:
        prev_encoder = encoder
        first_read = False

    diff = encoder - prev_encoder

    # filtreleme
    try:
        filter_encoders(encoder)
    except:
        return

    tick_per_degree = 583
    total_degree = total_degree + (diff / tick_per_degree)

    rospy.loginfo("total degree: " + str(total_degree))

    prev_encoder = encoder

def cb(msg):
    global speed
    speed = msg.data

if __name__ == "__main__":
    args = sys.argv
    if len(args) == 1:
        port = "/dev/ttyUSB1"
    else:
        port = args[1]

    rospy.init_node('lift_controller')
    robot = minimal_modbus.LiftDevice(port, 9600)

    speed = 0 # RPM
    rospy.Subscriber("/liftSpeed",Float32,cb)

    diff = 0
    first_read = True
    total_degree = 0

    prev_encoder = None

    list_encode = []

    rate = rospy.Rate(100)
    print("started lift_controller node")

    while not rospy.is_shutdown():
        robot.write_speed_command(speed)
        # publish_encoder()
        rate.sleep()

    robot.write_speed_command(0)
    
