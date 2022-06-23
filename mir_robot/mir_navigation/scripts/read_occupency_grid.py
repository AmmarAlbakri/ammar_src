#!/usr/bin/env python3

from random import randrange
import time
import rospy
from nav_msgs.msg import OccupancyGrid

def cb(data):
    global new_map_ready
    global msg
    if not new_map_ready:
        msg.header = data.header
        msg.info = data.info

        width = data.info.width
        height = data.info.height

        # for i in data.data:
        #     if i == 100:
        #         msg.data.append(100)
        #     elif randrange(2) % 2  == 0:
        #         msg.data.append(25)
        #     else:
        #         msg.data.append(75)


        
        new_map_ready = True
 

if __name__ == "__main__":
    rospy.init_node('read_occupency_grid')
    rospy.Subscriber("/premap", OccupancyGrid, cb)
    # rospy.Subscriber("/premap_metadata", OccupancyGrid, cb)
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    # pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    new_map_ready = False
    rate = rospy.Rate(1)
    msg = OccupancyGrid()
    while not rospy.is_shutdown():
        if new_map_ready:
            pub.publish(msg)
            print(msg)
            print("Publish edited map")
            time.sleep(10)
            
