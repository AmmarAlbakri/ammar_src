#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import rospkg


def run_mission():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.set_param('/runSequence',False)

    r = rospkg.RosPack()
    path = r.get_path('amr_missions')
    with open(path+"/missions/mission.yaml", "r") as file:
        parsed_yaml = yaml.safe_load(file)

    goals = []
    # for i in range(parsed_yaml['mission1']['number_of_goals']):
    #     goals.append(parsed_yaml['mission1']['goal'+str(i+1)])
    for i in range(parsed_yaml['number_of_goals']):
        goals.append(parsed_yaml['goal'+str(i+1)])

    rospy.loginfo("Running mission; number of goals: " + str(parsed_yaml['number_of_goals']))

    for _goal in goals:
        print(goals)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = _goal["position"]["x"]
        goal.target_pose.pose.position.y = _goal["position"]["y"]
        goal.target_pose.pose.orientation.w = _goal["orientation"]["w"]
        goal.target_pose.pose.orientation.z = _goal["orientation"]["z"]

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            pass


if __name__ == "__main__":
    rospy.init_node('run_mission')
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    while not rospy.is_shutdown():
        if rospy.get_param('/runSequence',False):
            run_mission()
