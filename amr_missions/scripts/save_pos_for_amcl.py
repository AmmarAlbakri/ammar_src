#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Pose
import yaml
import rospkg
from tf.transformations import euler_from_quaternion, quaternion_from_euler


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

            (r, p, y) = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])

            new_pose = dict(
                {
                    "initial_pose_x": trans[0],
                    "initial_pose_y": trans[1],
                    "initial_pose_a": y
                }
            )
            r = rospkg.RosPack()
            path = r.get_path('amr_missions')
            with open(path+"/markers/lastPose.yaml", "w") as file:
                yaml.dump(new_pose, file)
                rospy.loginfo("Saved Position as last position")
            rospy.set_param('/saveAMCLPose', False)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node('save_pose_for_amcl')
    robot_pose = None
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        if rospy.get_param('/saveAMCLPose', False):
            save_pos()
