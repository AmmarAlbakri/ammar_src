import rospy,rosbag,tf,actionlib
import getpass
import os,yaml
from amr_docking.srv import RecordManualPath, RecordManualPathResponse
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist,PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def cmd_cb(msg):
    bag.write(msg=msg, topic="cmd_vel")


def is_same(left, right):
    if left.position.x - right.position.x > 0.001:
        return False
    if left.position.y - right.position.y > 0.001:
        return False
    if left.orientation.x - right.orientation.x > 0.001:
        return False
    if left.orientation.y - right.orientation.y > 0.001:
        return False
    if left.orientation.z - right.orientation.z > 0.001:
        return False
    if left.orientation.w - right.orientation.w > 0.001:
        return False
    return True


def start_record():
    global prev_odom
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
    except:
        pass
    pose.position.x = float("{0:.4f}".format(pose.position.x))
    pose.position.y = float("{0:.4f}".format(pose.position.y))

    pose.orientation.x = float("{0:.4f}".format(pose.orientation.x))
    pose.orientation.y = float("{0:.4f}".format(pose.orientation.y))
    pose.orientation.z = float("{0:.4f}".format(pose.orientation.z))
    pose.orientation.w = float("{0:.4f}".format(pose.orientation.w))

    new_odom = Pose()
    new_odom.position = pose.position
    new_odom.orientation = pose.orientation

    if prev_odom is None:
        prev_odom = new_odom
    elif is_same(prev_odom, new_odom):
        # print("same")
        pass
    else:
        # print("not same")
        poseStamped = PoseStamped()
        poseStamped.pose = new_odom
        header = rospy.Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.now()
        poseStamped.header = header
        poses.append(poseStamped)
    prev_odom = new_odom
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    path.poses = poses
    pub.publish(path)
    print("publishing path")


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
        pass


def read_poses():
    global path
    data = None
    poses = []
    with open("../markers/RecordedPath.yaml", "r") as file:
        data = yaml.load(file)
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


def trigger(req):
    global bag
    global sub
    global poses
    global state
    if req.action == "start":
        print("started recording")
        state = "recording"
        bag = rosbag.Bag("../bags/cmd_vel.bag", "w")
        sub = rospy.Subscriber('/cmd_vel', Twist, cmd_cb)
        poses = []
        prev_odom = None
        while state == "recording":
            start_record()
    elif req.action == "stop":
        print("stoped recording")
        state = "stop"
        bag.close()
        sub.unregister()
        data = dict()
        with open("../markers/RecordedPath.yaml", "w") as file:
            for i in range(len(poses)):
                data.update({
                    "pose"+str(i): {
                        "position": {
                            "x": poses[i].pose.position.x,
                            "y": poses[i].pose.position.y,
                            "z": poses[i].pose.position.z
                        },
                        "orientation": {
                            "x": poses[i].pose.orientation.x,
                            "y": poses[i].pose.orientation.y,
                            "z": poses[i].pose.orientation.z,
                            "w": poses[i].pose.orientation.w
                        }
                    }})
            data.update({"numberOfPoses": len(poses)})
            yaml.dump(data, file)
    elif req.action == "play":
        print("started playing")
        state = "playing"
        read_poses()
        username = getpass.getuser()
        os.chdir(
            f"/home/{username}/catkin_ws/src/mir_robot/mir_navigation/bags/")
        os.system('rosbag play cmd_vel.bag')


if __name__ == "__main__":
    rospy.init_node("save_cmd_vel_commands")
    bag = None
    listener = tf.TransformListener()
    sub = None
    prev_odom = None
    state = None
    poses = []
    s = rospy.Service('record_path', RecordManualPath, trigger)
    pub = rospy.Publisher("/recorded_path", Path, queue_size=10)
    while not rospy.is_shutdown():
        pass
