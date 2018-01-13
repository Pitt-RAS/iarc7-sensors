#!/usr/bin/env python
import sys
import rospy

from iarc7_msgs.msg import BoolStamped
from iarc7_msgs.msg import LandingGearContactsStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

from iarc7_safety.SafetyClient import SafetyClient

def switch_callback(msg):
    global last_switch_message
    last_switch_message = msg

def odom_callback(msg):
    global last_height
    last_height = msg.pose.pose.position.z

last_switch_message = None
last_height = 0.0
height_indicates_landed = False

if __name__ == '__main__':
    rospy.init_node('landing_detector')
    absolute_velocity_pub = rospy.Publisher('absolute_vel', TwistWithCovarianceStamped, queue_size=0)
    landing_detected_pub = rospy.Publisher('landing_detected', BoolStamped, queue_size=0)
    rospy.Subscriber('landing_gear_contact_switches', LandingGearContactsStamped, switch_callback)
    rospy.Subscriber('odometry/filtered', Odometry, odom_callback)

    rate = rospy.Rate(rospy.get_param('~update_rate'))

    switch_expected_update_rate = rospy.get_param('~switch_expected_update_rate')
    switch_update_lag_tolerance = rospy.get_param('~switch_update_lag_tolerance')
    switch_startup_timeout = rospy.get_param('~switch_startup_timeout')
    landing_detected_height = rospy.get_param('~landing_detected_height')
    takeoff_detected_height = rospy.get_param('~takeoff_detected_height')
    takeoff_detected_override_height = rospy.get_param('~takeoff_detected_override_height')

    # Wait for a valid timestamp
    while rospy.Time.now() == rospy.Time(0):
        if rospy.is_shutdown():
            raise rospy.exceptions.ROSInterruptException('No valid timestamp before shutdown')
        rate.sleep()

    # Make sure a message is recieved and published before connecting
    # to safety
    start_time = rospy.Time.now()
    while True:
        assert((rospy.Time.now() - start_time) < rospy.Duration(20.0))
        if last_switch_message is not None:
            break
        if rospy.is_shutdown():
            raise rospy.exceptions.ROSInterruptException('No message before timeout')
        rate.sleep()

    safety_client = SafetyClient('landing_detector')
    assert(safety_client.form_bond())

    while not rospy.is_shutdown():
        if (safety_client.is_fatal_active()
           or safety_client.is_safety_active()):
            # We don't have a safety response for this node so just exit
            break

        # Make sure we are within 10Hz of the 30Hz target update rate for switch updates
        if (rospy.Time.now() - last_switch_message.header.stamp) > rospy.Duration(1.0/(30.0-10.0)):
            rospy.logwarn_throttle(1.0,
                'Landing Detector is not receiving switch messages fast enough')



        if height_indicates_landed:
            height_indicates_landed = last_height > 0.5
        else:
            height_indicates_landed = last_height < 0.3

        landing_detected = ((last_switch_message.front
                            + last_switch_message.back
                            + last_switch_message.left
                            + last_switch_message.right
                            + height_indicates_landed) > 3
                            and not last_height > 0.6)


        if landing_detected:
            # Publish the fact that the velocity is 0
            vel_msg = TwistWithCovarianceStamped()
            vel_msg.header.frame_id = 'map'
            vel_msg.header.stamp = rospy.Time.now()
            absolute_velocity_pub.publish(vel_msg)

        # Publish the ground state message
        state_msg = BoolStamped()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.data = landing_detected
        landing_detected_pub.publish(state_msg)

        rate.sleep()
