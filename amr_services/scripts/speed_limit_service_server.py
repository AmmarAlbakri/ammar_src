#!/usr/bin/env python3
import rospy
from amr_services.srv import SpeedLimit,SpeedLimitResponse


def change_speed_limit(req):
    global speed_limit_state
    global speed_limit_value
    speed_limit_state = req.speed_limit_state
    speed_limit_value = req.speed_limit_value
    rospy.set_param("/speed_limit_state",speed_limit_state)
    rospy.set_param("/speed_limit_value",speed_limit_value)
    if req.speed_limit_state == 1:
        print("speed_limit_state ON")
        return SpeedLimitResponse(1)
    elif req.speed_limit_state == 0:
        print("speed_limit_state OFF")
        return SpeedLimitResponse(1)
    else:
        print("speed_limit_state ERROR")
        return SpeedLimitResponse(-1)


if __name__ == "__main__":
    speed_limit_state = True
    speed_limit_value = 0.3
    rospy.set_param("/speed_limit_value",speed_limit_state)
    rospy.set_param("/speed_limit_state",speed_limit_value)

    rospy.init_node('speed_limit_server')
    s = rospy.Service('speed_limit', SpeedLimit, change_speed_limit)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
