#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import rospkg

def movebase_client():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    r = rospkg.RosPack()
    path = r.get_path('amr_missions') 
    with open(path +"/markers/goal.yaml", "r") as file:
        parsed_yaml = yaml.safe_load(file)
        
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = parsed_yaml["position"]["x"]
    goal.target_pose.pose.position.y = parsed_yaml["position"]["y"]
    goal.target_pose.pose.orientation.w = parsed_yaml["orientation"]["w"]
    goal.target_pose.pose.orientation.z = parsed_yaml["orientation"]["z"]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.set_param('/sendGoal',False)
        return client.get_result()

if __name__ == "__main__":
    rospy.init_node('save_pos_as_goal')
    pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=10)
    # rospy.Subscriber("/move_base_simple/goal",PoseStamped,cb)
    while not rospy.is_shutdown():
        if rospy.get_param('/sendGoal',False):
            print(movebase_client())
