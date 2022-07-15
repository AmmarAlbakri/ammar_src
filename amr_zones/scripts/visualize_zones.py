#!/usr/bin/env python3

import rospy, rospkg
import yaml
from geometry_msgs.msg import Point, Polygon, PolygonStamped

def new_zone_cb(data):
    read_zones()

def read_zones():
    global zones
    global r
    path = r.get_path('amr_zones')
    with open(path + "/zones/zones.yaml", 'r') as stream:
        parsed_yaml = yaml.safe_load(stream)
        zone_number = parsed_yaml["zone_number"]
        zones = []
    for i in range(zone_number):
        polygon = Polygon()
        for j in range(parsed_yaml['zone'+str(i+1)]['point_number']):
            point = Point(
                parsed_yaml["zone"+str(i+1)]["point"+str(j+1)]["x"],
                parsed_yaml["zone"+str(i+1)]["point"+str(j+1)]["y"],
                parsed_yaml["zone"+str(i+1)]["point"+str(j+1)]["z"]
            )
            polygon.points.append(point)
        
        zone = {'polygon': polygon,
                'zone_id': i+1
                }
        zones.append(zone)

def publish_zones():
    seq = 1
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # print("Visualizing " + str(len(zones)) + " zones")
        for zone in zones:
            msg = PolygonStamped()
            msg.polygon = zone["polygon"]
            msg.header.frame_id = 'map'
            msg.header.stamp = rospy.Time.now()
            msg.header.seq = seq = seq + 1
            pub_all.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('visualize_zones')
    r = rospkg.RosPack()
    zones = []
    read_zones()
    rospy.Subscriber("/new_zone", Polygon, new_zone_cb)
    pub_all = rospy.Publisher("/visualize_zone", PolygonStamped, queue_size=10)
    publish_zones()
