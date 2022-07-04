#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Polygon
from mir_msgs.msg import Zone
import yaml
import rospkg


def new_zone_cb(data):
    read_zones()

def read_zones():
    global zones
    global zone_number
    global r
    path = r.get_path('mir_navigation')

    with open(path + "/zones/zones.yaml", 'r') as stream:
        parsed_yaml = yaml.safe_load(stream)
        zone_number = parsed_yaml["zone_number"]
        zones = []
    for i in range(zone_number):
        polygon = Polygon()
        for j in range(parsed_yaml["zone"+str(i+1)]['point_number']):
            point = Point(
                parsed_yaml["zone"+str(i+1)]["point"+str(j+1)]["x"],
                parsed_yaml["zone"+str(i+1)]["point"+str(j+1)]["y"],
                parsed_yaml["zone"+str(i+1)]["point"+str(j+1)]["z"]
            )
            polygon.points.append(point)
        
        zone = {'polygon': polygon,
                'zone_id': i+1,
                'zone_type':  parsed_yaml["zone"+str(i+1)]["zone_type"]
                }
        zones.append(zone)


def publish_zones():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for zone in zones:
            msg = Zone()
            msg.polygon = zone["polygon"]
            msg.zone_id = zone["zone_id"]
            msg.zone_type = zone["zone_type"]
            pub_all.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('publish_zones')
    r = rospkg.RosPack()
    zone_number = 0
    zones = []
    read_zones()
    rospy.Subscriber("/new_zone", Polygon, new_zone_cb)
    pub_all = rospy.Publisher("/zones", Zone, queue_size=10)
    publish_zones()
