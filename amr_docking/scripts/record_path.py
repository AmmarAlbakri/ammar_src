import rospy
import tf
import rospkg
import yaml
from amr_docking.srv import RecordManualPath, RecordManualPathResponse
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
import os


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
    if state != "recording":
        return
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
    path.header = rospy.Header()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    path.poses = poses
    pub.publish(path)


def trigger(req):
    global poses
    global prev_odom
    global state

    if req.action == "start":
        print("start path recording")
        prev_odom = None
        poses = []
        state = "recording"
        return RecordManualPathResponse("started")

    elif req.action == "stop":
        print("stop path recording")
        print(poses)
        state = "stoping"
        data = dict()
        pkg_path = rospkg.RosPack().get_path('amr_docking')
        with open(pkg_path+"/markers/RecordedPath.yaml", "w") as file:
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
        return RecordManualPathResponse("saved " + str(len(poses)) + " number of poses")

    elif req.action == "play":
        os.system("rosrun amr_docking run_recorded_path.py")
        return RecordManualPathResponse("recording played")


if __name__ == "__main__":
    rospy.init_node("record_path")
    # rospy.Subscriber("/odom", Odometry, odom_cb)
    s = rospy.Service('record_path', RecordManualPath, trigger)
    listener = tf.TransformListener()
    pub = rospy.Publisher("/recorded_path", Path, queue_size=100)
    state = None
    prev_odom = None
    poses = []
    while not rospy.is_shutdown():
        start_record()
