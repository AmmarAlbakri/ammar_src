#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int16
import yaml
import rospkg


def new_point_cb(data):
    rospy.loginfo("new point rec")
    read_waypoints()


def read_waypoints():
    global points
    global point_number
    path = rospkg.RosPack().get_path('mir_navigation')

    with open(path + "/missions/mission.yaml", 'r') as stream:
        parsed_yaml = yaml.safe_load(stream)
        point_number = parsed_yaml["number_of_goals"]
        points = PoseArray()
    for i in range(point_number):
        point = Pose()
        point.position.x = parsed_yaml["goal"+str(i+1)]["position"]["x"]
        point.position.y = parsed_yaml["goal"+str(i+1)]["position"]["y"]
        point.position.z = 0
        points.poses.append(point)


def publish_waypoints():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        points.header.frame_id = "map"
        pub_all.publish(points)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('publish_waypoints')
    point_number = 0
    points = PoseArray()
    read_waypoints()
    rospy.Subscriber("/new_waypoint", Int16, new_point_cb)
    pub_all = rospy.Publisher("/waypoints", PoseArray, queue_size=10)
    publish_waypoints()
