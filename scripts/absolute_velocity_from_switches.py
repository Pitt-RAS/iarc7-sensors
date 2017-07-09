#!/usr/bin/env python
import sys
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
    global last_msg_stamp
    last_msg_stamp = msg.header.stamp

last_msg_stamp = None

if __name__ == '__main__':
    rospy.init_node('absolute_vel_from_switches')
    rospy.Subscriber('landing_gear_contact_switches', LandingGearContactsStamped, callback)
    pub = rospy.Publisher('absolute_vel', TwistWithCovarianceStamped, queue_size=0)

    rate = rospy.Rate(100)

    # Wait for a valid timestamp
    while rospy.Time.now() == rospy.Time(0):
        if rospy.is_shutdown():
            raise rospy.exceptions.ROSInterruptException('No valid timestamp before shutdown')

    # Make sure a message is recieved and published before connecting
    # to safety
    start_time = rospy.Time.now()
    while True:
        assert((rospy.Time.now() - start_time) < rospy.Duration(10.0))
        if last_msg_stamp is not None:
            break
        if rospy.is_shutdown():
            raise rospy.exceptions.ROSInterruptException('No message before timeout')
        rate.sleep()

    safety_client = SafetyClient('absolute_switch_velocity_publisher')
    assert(safety_client.form_bond())

    while not rospy.is_shutdown():
        if (safety_client.is_fatal_active()
           or safety_client.is_safety_active()):
            # We don't have a safety response for this node so just exit
            break
        # Make sure we are within 10Hz of the 50Hz target update rate
        if (rospy.Time.now() - last_msg_stamp) > rospy.Duration(1.0/(30.0-10.0)):
            rospy.logwarn('Absolute velocity from contact switches cant hit update rate')
        rate.sleep()
