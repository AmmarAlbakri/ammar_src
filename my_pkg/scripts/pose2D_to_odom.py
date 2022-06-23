#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D 
from nav_msgs.msg import Odometry
import tf


def callback(data):
    global x
    global y
    global theta
    x=data.x
    y=data.y
    theta=data.theta


def talker():
    global x
    global y
    global theta
    pub = rospy.Publisher('laser_odom', Odometry, queue_size=10)
    rate = rospy.Rate(60) # 10hz
    while not rospy.is_shutdown():
        msg_quat=tf.transformations.quaternion_from_euler(0, 0, theta)
        msg=Odometry()
        msg.header.stamp=rospy.Time.now()
        msg.header.frame_id="odom"
        msg.child_frame_id="base_footprint"
        msg.pose.pose.position.x=x
        msg.pose.pose.position.y=y 
        msg.pose.pose.position.z=0
        msg.pose.pose.orientation.x=msg_quat[0]
        msg.pose.pose.orientation.y=msg_quat[1]
        msg.pose.pose.orientation.z=msg_quat[2]
        msg.pose.pose.orientation.w=msg_quat[3]
        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        x=0
        y=0
        theta=0
        rospy.init_node('laser_odom', anonymous=True)
        rospy.Subscriber('pose2D', Pose2D, callback)
        talker()

    except rospy.ROSInterruptException:
        pass
    
