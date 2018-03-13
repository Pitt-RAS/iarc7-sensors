#!/usr/bin/env python

"""
Roomba State Estimator: estimates state of all visible roombas

"""

import rospy
import math

from iarc7_safety.SafetyClient import SafetyClient

from iarc7_msgs.msg import (OdometryArray, 
                            RoombaStateStamped, 
                            RoombaStateStampedArray)

class RoombaStates(object):
    MOVING_FORWARD = 1
    TURNING_AROUND = 2
    UNKNOWN = 3

class Roomba(object): 
    def __init__(self, roomba_id):
        self._state = RoombaStates.UNKNOWN
        self._roomba_id = roomba_id
        self._speed_indicates_turning = False
        self._turning_detected_speed = rospy.get_param('~turning_detected_speed')
        self._moving_detected_speed = rospy.get_param('~moving_detected_speed')

    def __eq__(self, roomba): 
        return self._roomba_id == roomba._roomba_id

    def update_state(self, odometry): 
        roomba_x_velocity = odometry.twist.twist.linear.x
        roomba_y_velocity = odometry.twist.twist.linear.y
        last_speed = math.sqrt(roomba_x_velocity**2 + roomba_y_velocity**2)
        
        if self._speed_indicates_turning:
            self._speed_indicates_turning = last_speed < self._moving_detected_speed
        else:
            self._speed_indicates_turning = last_speed < self._turning_detected_speed

        if self._speed_indicates_turning: 
            self._state = RoombaStates.TURNING_AROUND
        else: 
            self._state = RoombaStates.MOVING_FORWARD

    def is_turning_around(self):
        return self._state == RoombaStates.TURNING_AROUND

    def is_moving_forward(self): 
        return self._state == RoombaStates.MOVING_FORWARD

    def state_unknown(self): 
        return self._state == RoombaStates.UNKNOWN

def roomba_callback(msg): 
    global roomba_odoms
    roomba_odoms = msg

roomba_odoms = None

if __name__ == '__main__':
    rospy.init_node('roomba_state_estimator')

    roomba_dict = {}

    state_pub = rospy.Publisher('roomba_states', RoombaStateStampedArray, queue_size=0)
    rospy.Subscriber('roombas', OdometryArray, roomba_callback)

    rate = rospy.Rate(rospy.get_param('~roomba_state_estimator_update_rate'))
    startup_timeout = rospy.get_param('~roomba_state_estimator_start_up_timeout')

    # Wait for a valid timestamp
    while rospy.Time.now() == rospy.Time(0):
        if rospy.is_shutdown():
            raise rospy.exceptions.ROSInterruptException('No valid timestamp before shutdown')
        rate.sleep()

    # Make sure a message is recieved and published before connecting
    # to safety
    start_time = rospy.Time.now()
    while roomba_odoms is None: 
        assert((rospy.Time.now() - start_time) < rospy.Duration(startup_timeout))
        if rospy.is_shutdown():
            raise rospy.exceptions.ROSInterruptException('No message before shutdown')
        rate.sleep()

    safety_client = SafetyClient('roomba_state_estimator')
    assert(safety_client.form_bond())

    while not rospy.is_shutdown():
        if (safety_client.is_fatal_active()
           or safety_client.is_safety_active()):
            # We don't have a safety response for this node so just exit
            break

        for odom in roomba_odoms.data: 
            roomba_id = odom.child_frame_id
            
            if not odom.child_frame_id in roomba_dict: 
                roomba = Roomba(roomba_id)
                roomba_dict[roomba_id] = roomba
            else: 
                roomba = roomba_dict.get(roomba_id)

            roomba.update_state(odom)

            state_msg = RoombaStateStamped()
            roomba_states_msg = RoombaStateStampedArray()

            state_msg.header.stamp = rospy.Time.now()
            state_msg.roomba_id = roomba_id
            state_msg.moving_forward = roomba.is_moving_forward()
            state_msg.turning = roomba.is_turning_around()

            roomba_states_msg.roombas.append(state_msg)

        state_pub.publish(roomba_states_msg)

        rate.sleep()
