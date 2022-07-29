import rospy
import actionlib
import rospkg
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import os

def send_goal(pose):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose.pose.position.x
    goal.target_pose.pose.position.y = pose.pose.position.y
    goal.target_pose.pose.orientation.w = pose.pose.orientation.w
    goal.target_pose.pose.orientation.z = pose.pose.orientation.z
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print("next goal")


def read_poses():
    global path
    data = None
    poses = []
    pkg_path = rospkg.RosPack().get_path('amr_docking')
    with open(pkg_path+"/markers/RecordedPath.yaml", "r") as file:
        data = yaml.safe_load(file)
    if int(data["numberOfPoses"]) == 0:
        print("no recorded path to follow")
        return
    for i in range(int(data["numberOfPoses"])):
        pose = PoseStamped()
        pose.pose.position.x = data["pose"+str(i)]["position"]["x"]
        pose.pose.position.y = data["pose"+str(i)]["position"]["y"]
        pose.pose.orientation.x = data["pose"+str(i)]["orientation"]["x"]
        pose.pose.orientation.y = data["pose"+str(i)]["orientation"]["y"]
        pose.pose.orientation.z = data["pose"+str(i)]["orientation"]["z"]
        pose.pose.orientation.w = data["pose"+str(i)]["orientation"]["w"]

        header = rospy.Header()
        header.frame_id = "map"
        header.seq = i+1
        header.stamp = rospy.Time.now()
        pose.header = header

        poses.append(pose)
    path = Path()
    path.header.frame_id = "map"
    path.poses = poses

    send_goal(poses[0])
    # rospy.set_param("/speed_limit_value",0.5)
    # send_goal(poses[int(len(poses)/2)])
    # send_goal(poses[-1])
    # rospy.set_param("/speed_limit_value",1.5)
    rospy.set_param("/correct_orientation",True)

    # while not rospy.is_shutdown():
    #     path.header.stamp = rospy.Time.now()
    #     pub.publish(path)


if __name__ == "__main__":
    rospy.init_node("run_recorded_path")
    pub = rospy.Publisher("/recorded_path", Path, queue_size=10)
    pub_plan = rospy.Publisher(
        "/move_base_node/SBPLLatticePlanner/plan", Path, queue_size=10)
    path = None
    read_poses()
