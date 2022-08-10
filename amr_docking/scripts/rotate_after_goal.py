#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionGoal
from tf.transformations import euler_from_quaternion


def new_goal(msg):
    global goal_degree
    goal = msg.goal.target_pose.pose
    quat = [goal.orientation.x, goal.orientation.y,
            goal.orientation.z, goal.orientation.w]
    goal_degree = euler_from_quaternion(quat)[2] * 180 / 3.14
    print("goal at degree:" + str(goal_degree))


def goal_reached(msg):
    if msg.status.status == 3:
        print("goal reached")
        rospy.set_param("/goal_degree", goal_degree)


if __name__ == "__main__":
    rospy.init_node("rotate_after_goal")
    goal_degree = None
    rospy.Subscriber("/move_base/goal",
                     MoveBaseActionGoal, new_goal)
    rospy.Subscriber("/move_base/result",
                     MoveBaseActionResult, goal_reached)
    while not rospy.is_shutdown():
        rospy.spin()
