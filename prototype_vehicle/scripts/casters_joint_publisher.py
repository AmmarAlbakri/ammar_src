#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState


def casters_joint_publisher():
    rospy.init_node('casters_joint_publisher.py')
    prefix = rospy.get_param('~prefix', '')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    r = rospy.Rate(0.5)
    r.sleep()
    
    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = [
            # prefix + 'left_wheel_joint',
            # prefix + 'right_wheel_joint',
            prefix + 'fl_caster_rotation_joint',
            prefix + 'fl_caster_wheel_joint',
            prefix + 'fr_caster_rotation_joint',
            prefix + 'fr_caster_wheel_joint',
            prefix + 'bl_caster_rotation_joint',
            prefix + 'bl_caster_wheel_joint',
            prefix + 'br_caster_rotation_joint',
            prefix + 'br_caster_wheel_joint',
            'lift_joint',
        ]
        js.position = [0.0 for _ in js.name]
        js.velocity = [0.0 for _ in js.name]
        js.effort = [0.0 for _ in js.name]
        pub.publish(js)
        r.sleep()


if __name__ == '__main__':
    try:
        casters_joint_publisher()
    except rospy.ROSInterruptException:
        pass