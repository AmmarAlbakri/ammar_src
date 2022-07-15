#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, Polygon
import yaml
import rospkg


def clicked_point_cb(data):
    global points_clicked
    global points
    global pub
    points_clicked += 1
    points.append(data.point)
    print("New point")
    print(data.point)
    if points_clicked == 5:
        print("New zone published")
        msg = Polygon()
        msg.points = points
        save_zone(msg)
        pub.publish(msg)
        points_clicked = 0
        points = []


def save_zone(zone):
    r = rospkg.RosPack()
    path = r.get_path('amr_zones')
    with open(path+"/zones/zones.yaml", 'r') as stream:
        parsed_yaml = yaml.safe_load(stream)
        # print(parsed_yaml)
        parsed_yaml["zone_number"] += 1
        new_zone = dict({
            'zone_id': parsed_yaml["zone_number"],
            'zone_type': 1,
            'point_number': len(zone.points),
            # 'point1': {'x': zone.points[0].x,
            #             'y': zone.points[0].y,
            #             'z': zone.points[0].z},
            # 'point2': {'x': zone.points[1].x,
            #            'y': zone.points[1].y,
            #            'z': zone.points[1].z},
            # 'point3': {'x': zone.points[2].x,
            #            'y': zone.points[2].y,
            #            'z': zone.points[2].z},
            # 'point4': {'x': zone.points[3].x,
            #            'y': zone.points[3].y,
            #            'z': zone.points[3].z},
        })
        for i in range(len(zone.points)):
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
    points_clicked = 0
    points = []
    rospy.init_node('create_zones')
    rospy.Subscriber("/clicked_point", PointStamped, clicked_point_cb)
    pub = rospy.Publisher("/new_zone", Polygon, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
