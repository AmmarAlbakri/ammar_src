#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, Polygon
import yaml
import rospkg
from mir_msgs.msg import NewZone


def newZonecb(data):
    global pub
    msg = Polygon()
    msg.points = data.points
    save_zone(data)
    pub.publish(msg)
    print("New zone published")


def save_zone(zone):
    r = rospkg.RosPack()
    path = r.get_path('mir_navigation')
    with open(path+"/zones/zones.yaml", 'r') as stream:
        parsed_yaml = yaml.safe_load(stream)
        # print(parsed_yaml)
        parsed_yaml["zone_number"] += 1
        new_zone = dict({
            'zone_id': parsed_yaml["zone_number"],
            'zone_type': zone.type,
            'point_number': zone.point_count,
        })
        for i in range(zone.point_count):
            new_zone.update({
                'point'+str(i+1):
                {
                    'x': zone.points[i].x,
                    'y': zone.points[i].y,
                    'z': zone.points[i].z
                }
            })
        parsed_yaml["zone"+str(parsed_yaml["zone_number"])] = new_zone
    with open(path+"/zones/zones.yaml", 'w') as stream:
        yaml.dump(parsed_yaml, stream)


if __name__ == "__main__":
    rospy.init_node('create_zones')
    rospy.Subscriber("/create_new_zone", NewZone, newZonecb)
    pub = rospy.Publisher("/new_zone", Polygon, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
