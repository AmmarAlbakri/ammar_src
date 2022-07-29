import rospy
from sensor_msgs.msg import Imu
import tf
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist


def cb(msg):
    global yaw_degree
    quat = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw_degree = euler[2]*180/3.14
    # print(euler[2]*180/3.14)


def odom_cb(msg):
    global yaw_degree
    quat = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw_degree = euler[2]*180/3.14
    # print(euler[2]*180/3.14)


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


def rotate_to(end):
    msg = Twist()
    msg.linear.x = 0
    start = yaw_degree
    rotate_speed = -0.5
    print("Starting at:" + str(start))
    print("Ending at:" + str(end))

    while abs(yaw_degree - end) > 1:
        if start >= 0 and end >= 0:
            if start > end:
                msg.angular.z = rotate_speed
            else:
                msg.angular.z = -rotate_speed

        elif start <= 0 and end <= 0:
            if start > end:
                msg.angular.z = rotate_speed
            else:
                msg.angular.z = -rotate_speed

        elif start >= 0 and end <= 0:
            if start+abs(end) < 180:
                msg.angular.z = rotate_speed
            else:
                msg.angular.z = -rotate_speed

        elif start <= 0 and end >= 0:
            if abs(start) + end > 180:
                msg.angular.z = rotate_speed
            else:
                msg.angular.z = -rotate_speed
        print(yaw_degree)
        cmd_vel.publish(msg)

    print("Finish")
    cmd_vel.publish(Twist())
    rospy.set_param("/goal_degree", 999)


if __name__ == "__main__":
    rospy.init_node("read_yaw_from_imu")
    # rospy.Subscriber("/imu/data",Imu,cb)
    # rospy.Subscriber("/odometry/filtered",Odometry,odom_cb)
    rospy.Subscriber("/robot_pose", Pose, pose_cb)
    pub = rospy.Publisher("/yaw_degree", Float32, queue_size=100)
    cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    rate = rospy.Rate(60)
    yaw_degree = None
    while not rospy.is_shutdown():
        if yaw_degree is not None:
            pub.publish(yaw_degree)
        if rospy.get_param("/goal_degree", 999) != 999 and yaw_degree is not None:
            rotate_to(rospy.get_param("/goal_degree"))
        rate.sleep()
