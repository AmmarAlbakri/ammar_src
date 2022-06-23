import rospy
from sensor_msgs.msg import LaserScan


def callback(veri):

    global pub
    msg = LaserScan()
    msg.header = veri.header
    msg.angle_min = veri.angle_min
    msg.angle_max = veri.angle_max
    msg.angle_increment = veri.angle_increment
    msg.time_increment = veri.time_increment
    msg.scan_time = veri.scan_time
    msg.range_min = veri.range_min
    msg.range_max = veri.range_max
    msg.ranges = []

    for i in range(len(veri.ranges)):
        if veri.ranges[i] == float('inf'):
            msg.ranges.append(veri.range_max)
        else:
            msg.ranges.append(veri.ranges[i])
    pub.publish(msg)
     
if __name__ == '__main__':
    try:
        rospy.init_node('laser_deneme', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, callback)
        pub = rospy.Publisher('scan_pub',LaserScan, queue_size=10)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
