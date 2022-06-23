import rospy
from sensor_msgs.msg import LaserScan

def cb(data):
    print(len(data.ranges))


if __name__=="__main__":
    rospy.init_node("temp")
    rospy.Subscriber("/scan",LaserScan,cb)
    rospy.spin()