import rospy
import tf
from geometry_msgs.msg import Pose, Twist
from numpy import average


def pose_cb(msg):
    global yaw_degree
    quat = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw_degree = euler[2]*180/3.14


def get_direction(start, end):
    if start >= 0 and end >= 0:
        if start > end:
            return "CW"
        else:
            return "CCW"

    elif start <= 0 and end <= 0:
        if start > end:
            return "CW"
        else:
            return "CCW"

    elif start >= 0 and end <= 0:
        if start+abs(end) < 180:
            return "CW"
        else:
            return "CCW"

    elif start <= 0 and end >= 0:
        if abs(start) + end > 180:
            return "CW"
        else:
            return "CCW"


def rotate_to(end):
    msg = Twist()
    msg.linear.x = 0
    print("Starting at:" + str(yaw_degree))
    print("Ending at:" + str(end))

    while abs(yaw_degree - end) > 0.5:
        rotate_speed = max(abs(yaw_degree - end) / 150, 0.1)  # 1.2 --- 0.1
        if get_direction(yaw_degree, end) == "CW":
            rotate_speed = - rotate_speed
        msg.angular.z = rotate_speed
        print(yaw_degree)
        cmd_vel.publish(msg)

    cmd_vel.publish(Twist())
    rospy.set_param("/goal_degree", 999)


if __name__ == "__main__":
    rospy.init_node("rotate_to")
    rospy.Subscriber("/robot_pose", Pose, pose_cb)
    cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    rate = rospy.Rate(60)
    yaw_degree = None
    while not rospy.is_shutdown():
        if rospy.get_param("/goal_degree", 999) != 999 and yaw_degree is not None:
            rotate_to(rospy.get_param("/goal_degree"))
        rate.sleep()
