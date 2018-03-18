#!/usr/bin/env python

"""
Roomba State Estimator: estimates state of all visible roombas

"""
import rospy
import math
import threading

from nav_msgs.msg import Odometry

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
        self._last_pos = None
        self._last_odom = None
        self._turning_detected_speed = rospy.get_param('~turning_detected_speed')
        self._moving_detected_speed = rospy.get_param('~moving_detected_speed')
        self._max_expected_roomba_vel = rospy.get_param('~roomba_expected_vel')

    def update_state(self): 
        roomba_x_velocity = self._last_odom.twist.twist.linear.x
        roomba_y_velocity = self._last_odom.twist.twist.linear.y
        last_speed = math.sqrt(roomba_x_velocity**2 + roomba_y_velocity**2)
        
        if self._speed_indicates_turning:
            self._speed_indicates_turning = last_speed < self._moving_detected_speed
        else:
            self._speed_indicates_turning = last_speed < self._turning_detected_speed

        if self._speed_indicates_turning: 
            self._state = RoombaStates.TURNING_AROUND
        else: 
            self._state = RoombaStates.MOVING_FORWARD

    def update_odom(self, curr_pos): 
        if curr_pos is None: 
            raise ValueError('a bad roomba odometry message was provided')
        if self._last_pos is None: 
            self._last_pos = curr_pos
            return curr_pos
        else: 
            time_delta = (curr_pos.header.stamp-self._last_pos.header.stamp).to_sec()
            if time_delta <= 0: 
                raise RuntimeError('invalid time delta in update_odom for roomba_state_estimator')

            new_pose = curr_pos.pose.pose.position
            last_pose = self._last_pos.pose.pose.position

            x_vel = (new_pose.x - last_pose.x)/time_delta
            y_vel = (new_pose.y - last_pose.y)/time_delta

            # caps velocity to max expected velocity
            vel = math.sqrt(x_vel**2 + y_vel**2)

            if vel > self._max_expected_roomba_vel:
                rospy.logerr('Calculated roomba velocity {} above expected velocity'.format(vel))

                x_vel = x_vel * (self._max_expected_roomba_vel/vel)
                y_vel = y_vel * (self._max_expected_roomba_vel/vel)

            self._last_pos = curr_pos

            msg = Odometry()
            msg.child_frame_id = self._roomba_id
            msg.twist.twist.linear.x = x_vel
            msg.twist.twist.linear.y = y_vel
            msg.pose.pose.position = new_pose
            msg.header.stamp = rospy.Time.now()

            self._last_odom = msg

            return msg

    def is_turning_around(self):
        return self._state == RoombaStates.TURNING_AROUND

    def is_moving_forward(self): 
        return self._state == RoombaStates.MOVING_FORWARD

    def state_unknown(self): 
        return self._state == RoombaStates.UNKNOWN

class RoombaStateEstimator(objec): 
    def __init__(self): 
        rospy.init_node('roomba_state_estimator')

        self._roomba_poses = None

        self._state_pub = rospy.Publisher('roomba_states', RoombaStateStampedArray, queue_size=0)
        self._odom_pub = rospy.Publisher('roombas', OdometryArray, queue_size=0)

        self._lock = threading.Lock()

        rospy.Subscriber('roomba_observations', OdometryArray, roomba_callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down roomba state estimator");

    def roomba_callback(self, msg): 
        self._roomba_poses = msg

        roomba_odoms_msg = OdometryArray()
        roomba_states_msg = RoombaStateStampedArray()

        for pos in roomba_pos.data: 
            roomba_id = pos.child_frame_id
            
            if not roomba_id in roomba_dict: 
                roomba = Roomba(roomba_id)
                roomba_dict[roomba_id] = roomba
            else: 
                roomba = roomba_dict.get(roomba_id)

            roomba_odom = roomba.update_odom(pos)
            roomba_odoms_msg.data.append(roomba_odom)

            roomba.update_state()
            state_msg = RoombaStateStamped()
            state_msg.header.stamp = rospy.Time.now()
            state_msg.roomba_id = roomba_id
            state_msg.moving_forward = roomba.is_moving_forward()
            state_msg.turning = roomba.is_turning_around()

            roomba_states_msg.roombas.append(state_msg)

        self._state_pub.publish(roomba_states_msg)
        self._odom_pub.publish(roomba_odoms_msg)

if __name__ == '__main__':
    RoombaStateEstimator()
