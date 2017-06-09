#!/usr/bin/env python
import rospy

from iarc7_msgs.msg import LandingGearContactsStamped
from geometry_msgs.msg import TwistWithCovarianceStamped

from iarc7_safety.SafetyClient import SafetyClient

def callback(msg):
    if msg.front and msg.back and msg.left and msg.right:
        # We're on the ground
        out_msg = TwistWithCovarianceStamped()
        out_msg.header.frame_id = 'map'
        out_msg.header.stamp = msg.header.stamp
        pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('absolute_vel_from_switches')
    rospy.Subscriber('landing_gear_contact_switches', LandingGearContactsStamped, callback)
    pub = rospy.Publisher('absolute_vel', TwistWithCovarianceStamped, queue_size=0)

    safety_client = SafetyClient('absolute_switch velocity_publisher')
    assert(safety_client.form_bond())

    while not rospy.is_shutdown():
        if (self._safety_client.is_fatal_active()
           or self._safety_client.is_safety_active()):
            # We don't have a safety response for this node so just exit
           break

