#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16
import yaml
import rospkg


def save_pos():
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                '/map', '/base_footprint', rospy.Time(0))
            pose = Pose()
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            new_goal = dict(
                {'position':
                 {
                     'x': trans[0],
                     'y': trans[1],
                     'z': trans[2]
                 },
                    'orientation':
                    {
                    'x': rot[0],
                    'y': rot[1],
                    'z': rot[2],
                    'w': rot[3]
                 }}
            )
            r = rospkg.RosPack()
            path = r.get_path('mir_navigation')
            with open(path+"/markers/goal.yaml", "w") as file:
                yaml.dump(new_goal, file)
                rospy.loginfo("Saved Position as goal")
            with open(path+"/missions/mission.yaml", "r") as file:
                parsed_yaml = yaml.safe_load(file)
                parsed_yaml["number_of_goals"] += 1
                parsed_yaml['goal' +
                            str(parsed_yaml["number_of_goals"])] = new_goal
                rospy.loginfo("Added Position to mission")
            with open(path+"/missions/mission.yaml", 'w') as file:
                yaml.dump(parsed_yaml, file)
            rospy.set_param('/savePosition', False)
            pub = rospy.Publisher("/new_waypoint", Int16, queue_size=10)
            pub.publish(1)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)


def clean_positions():
    r = rospkg.RosPack()
    path = r.get_path('mir_navigation')
    with open(path+"/markers/goal.yaml", "w") as file:
        pass
    with open(path+"/missions/mission.yaml", "w") as file:
        parsed_yaml = {"number_of_goals": 0, "mission_name": "1_goal_mission"}
        yaml.dump(parsed_yaml,file)
    pub = rospy.Publisher("new_waypoint",Int16)
    pub.publish(1)
    rospy.set_param('/clearPositions',False)

if __name__ == "__main__":
    rospy.init_node('save_pos_as_goal')
    robot_pose = None
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        if rospy.get_param('/savePosition', False):
            save_pos()
        if rospy.get_param('/clearPositions', False):
            clean_positions()
