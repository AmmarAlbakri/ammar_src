#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from mir_navigation.srv import ChangeSafety, ChangeSafetyResponse


def changeSafety(req):
    global safety
    if req.state == 1:
        safety = True
        rospy.set_param("/safety",True)
        print("Safety ON")
        return ChangeSafetyResponse(1)
    elif req.state == 0:
        safety = False
        rospy.set_param("/safety",False)
        print("Safety OFF")
        return ChangeSafetyResponse(1)
    else:
        print("Safety ERROR")
        return ChangeSafetyResponse(-1)


if __name__ == "__main__":
    safety = True
    rospy.set_param("/safety",True)

    rospy.init_node('safety_service_server')
    # safety_pub = rospy.Publisher("/safety", Bool, queue_size=10)
    s = rospy.Service('change_safety', ChangeSafety, changeSafety)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # safety_pub.publish(safety)
        rate.sleep()
