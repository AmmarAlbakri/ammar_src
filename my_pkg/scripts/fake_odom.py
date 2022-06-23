#!/usr/bin/python3
from std_msgs.msg import Int32, Float32
import rospy

if __name__ == "__main__":
    rospy.init_node('fake_odom')
    right_encoder_pub = rospy.Publisher('right_ticks', Float32, queue_size=100)
    left_encoder_pub = rospy.Publisher('left_ticks', Float32, queue_size=100)

    degree = 0
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        degree -= 0.1
        right_encoder_pub.publish(degree)
        left_encoder_pub.publish(degree)
        rate.sleep()
