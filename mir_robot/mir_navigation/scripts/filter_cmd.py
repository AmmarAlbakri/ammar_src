#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def cmd_cb(data):
    global pub

    safety = rospy.get_param("/safety",True)
    speed_limit_state = rospy.get_param("/speed_limit_state",True)
    speed_limit_value = rospy.get_param("/speed_limit_value",0.3)
    front_state = rospy.get_param("/front_state","NONE")
    back_state = rospy.get_param("/back_state","NONE")

    # print("Safety: " + str(safety))
    # print("speed_limit_state: " + str(speed_limit_state))
    msg = data
    if rospy.get_param('/emergencyStop',False):
        pub.publish(Twist())
        return

    if safety:
        if front_state == "DANGER" and data.linear.x > 0:
            pub.publish(Twist())
            # print("Front Danger")

        elif back_state == "DANGER" and data.linear.x < 0:
            pub.publish(Twist())
            # print("Back Danger")

        elif front_state == "WARNING" and data.linear.x > 0:
            msg.linear.x = round(data.linear.x,2) * 0.5
            msg.angular.z = round(data.angular.z,2) * 0.5
            if speed_limit_state == True:
                msg.linear.x = min(abs(msg.linear.x),speed_limit_value) * msg.linear.x/abs(msg.linear.x)
            pub.publish(msg)
            # print("Front Warning")

        elif back_state == "WARNING" and data.linear.x < 0:
            msg = data
            msg.linear.x = round(data.linear.x,2) * 0.5
            msg.angular.z = round(data.angular.z,2) * 0.5
            if speed_limit_state == True:
                msg.linear.x = min(abs(msg.linear.x),speed_limit_value) * msg.linear.x/abs(msg.linear.x)
            pub.publish(msg)
            # print("Back Warning")
        else:
            if speed_limit_state == True and msg.linear.x != 0:
                msg.linear.x = min(abs(msg.linear.x),speed_limit_value) * msg.linear.x/abs(msg.linear.x)
            pub.publish(data)
            # print("Normal")
    else:
        if speed_limit_state == True and msg.linear.x != 0:
                msg.linear.x = min(abs(msg.linear.x),speed_limit_value) * msg.linear.x/abs(msg.linear.x)
        pub.publish(data)
        # print("Normal")

# def safety_cb(data):
#     global safety
#     safety = data

if __name__ == "__main__":
    safety = True
    rospy.init_node('filter_cmd')
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/cmd_vel_unfiltered", Twist, cmd_cb)

    # rospy.Subscriber("/safety", Bool, safety_cb)
    # rospy.Subscriber("/front_safety", Bool, front_safety_cb)
    # rospy.Subscriber("/back_safety", Bool, back_safety_cb)

    rospy.spin()
