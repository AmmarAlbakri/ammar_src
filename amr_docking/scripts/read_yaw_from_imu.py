import rospy
from sensor_msgs.msg import Imu
import tf
from std_msgs.msg import Float32

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



if __name__ == "__main__":
    rospy.init_node("read_yaw_from_imu")
    rospy.Subscriber("/imu/data",Imu,cb)
    pub = rospy.Publisher("/yaw_degree",Float32,queue_size=100)
    rate = rospy.Rate(60)
    yaw_degree = None
    while not rospy.is_shutdown():
        if yaw_degree is not None:
            pub.publish(yaw_degree)
        rate.sleep()