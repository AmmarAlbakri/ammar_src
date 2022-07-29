#!/usr/bin/env python3

import rospy
from amr_zones.msg import Zone, ZoneState
import tf
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

def is_robot_in_zone(zone):
    points = []
    for point in zone.polygon.points:
        points.append(Point(point.x,point.y))
    polygon = Polygon(points)
    # point1 = zone.polygon.points[0]
    # point2 = zone.polygon.points[1]
    # point3 = zone.polygon.points[2]
    # point4 = zone.polygon.points[3]
    # min_x = min(point1.x, point2.x, point3.x, point4.x)
    # min_y = min(point1.y, point2.y, point3.y, point4.y)
    # max_x = max(point1.x, point2.x, point3.x, point4.x)
    # max_y = max(point1.y, point2.y, point3.y, point4.y)
    try:
        (trans, rot) = listener.lookupTransform(
            '/map', '/base_footprint', rospy.Time(0))
        robot_pose = Point(trans[0],trans[1])

        if polygon.contains(robot_pose):
        # if robot_pose['x'] >= min_x and robot_pose['x'] <= max_x and \
        #         robot_pose['y'] >= min_y and robot_pose['y'] <= max_y:
            # print("Robot is currently at:")
            # print(robot_pose)
            return True
        else:
            return False
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return False
    except:
        return False


def check_zone(zones):
    zone_state = ZoneState()
    # print("checking " + str(len(zones)) + " zones")
    for zone in zones:
        if is_robot_in_zone(zone):
            zone_state.included_zones.append(zone.zone_id)
    return zone_state


def zones_cb(data):
    new_zone = True
    for zone in zones:
        if zone.zone_id == data.zone_id:
            new_zone = False
    if new_zone:
        zones.append(data)


if __name__ == "__main__":
    rospy.init_node('check_zones')
    zones = []
    robot_pose = None
    rospy.Subscriber("/zones", Zone, zones_cb)
    pub = rospy.Publisher("/zone_state", ZoneState, queue_size=10)
    listener = tf.TransformListener()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        zone_state = check_zone(zones)
        pub.publish(zone_state)
        rate.sleep()
