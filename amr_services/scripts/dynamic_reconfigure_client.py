#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.client import Client

def callback(config):
    rospy.loginfo("Config set to {footprint}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = Client("/move_base_node/global_costmap", timeout=5, config_callback=callback)
    print(client.get_parameter_descriptions())
    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        client.update_configuration({"footprint":[[0.6,-0.3],[0.5,0.3],[-0.3,0.3],[-0.4,-0.2]]})
        r.sleep()