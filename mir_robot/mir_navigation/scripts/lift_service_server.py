#!/usr/bin/env python3
import rospy
from mir_navigation.srv import Lift, LiftResponse


def lift(req):
    global safety
    if req.action == 'UP':
        rospy.set_param("/lift",'UP')
        print("lift UP")
        return LiftResponse(1)
    elif req.action == 'DOWN':
        rospy.set_param("/lift",'DOWN')
        print("lift DOWN")
        return LiftResponse(1)
    else:
        print("Lift ERROR")
        return LiftResponse(-1)


if __name__ == "__main__":
    safety = True
    rospy.set_param("/lift",'')

    rospy.init_node('lift_service_server')
    s = rospy.Service('Lift', Lift, lift)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
