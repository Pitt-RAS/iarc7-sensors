#!/usr/bin/env python

"""
Roomba State Estimator: estimates state of all visible roombas

"""
import rospy
import math
import threading
import numpy as np

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
        self._roomba_id = roomba_id

        # odom estimating
        self._last_time = None
        self._last_odom = None
        self._max_expected_roomba_vel = rospy.get_param('~roomba_expected_vel')

        self._expected_moving_pose_diff = self._max_expected_roomba_vel * .03

        # state estimating
        self._state = RoombaStates.UNKNOWN
        self._speed_indicates_turning = False

        self._turning_detected_speed = rospy.get_param('~turning_detected_speed')
        self._moving_detected_speed = rospy.get_param('~moving_detected_speed')

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

        if self._last_time is None:
            self._last_time = curr_pos.header.stamp
            self._last_odom = curr_pos
            return curr_pos
        else:
            new_x = curr_pos.pose.pose.position.x
            new_y = curr_pos.pose.pose.position.y

            last_x = self._last_odom.pose.pose.position.x
            last_y = self._last_odom.pose.pose.position.y

            s = np.array([(new_x-last_x), (new_y-last_y)])

            x_var = self._last_odom.pose.covariance[0]
            y_var = self._last_odom.pose.covariance[7]

            # these 2 values should be equal
            xy_var = self._last_odom.pose.covariance[1]
            yx_var = self._last_odom.pose.covariance[6]

            sigma = np.array([[x_var, xy_var],[yx_var, y_var]])

            sigma_inv = np.linalg.inv(sigma)

            s_T = np.transpose(s)

            result = s_T.dot(sigma_inv).dot(s)

            rospy.logerr(self._roomba_id)
            rospy.logerr(str(result))

            if result > 10: 
                x_vel = .3
                y_vel = .3
            else: 
                x_vel = 0
                y_vel = 0

            msg = Odometry()
            msg.child_frame_id = self._roomba_id
            msg.twist.twist.linear.x = x_vel
            msg.twist.twist.linear.y = y_vel
            msg.pose = curr_pos.pose
            msg.header.stamp = rospy.Time.now()

            self._last_odom = msg
            self._last_time = curr_pos.header.stamp

            return msg

    def is_turning_around(self):
        return self._state == RoombaStates.TURNING_AROUND

    def is_moving_forward(self):
        return self._state == RoombaStates.MOVING_FORWARD

    def state_unknown(self):
        return self._state == RoombaStates.UNKNOWN

class RoombaStateEstimator(object):
    def __init__(self):
        rospy.init_node('roomba_state_estimator')

        self._roomba_dict = {}

        self._state_pub = rospy.Publisher('roomba_states', RoombaStateStampedArray, queue_size=0)
        self._odom_pub = rospy.Publisher('roombas', OdometryArray, queue_size=0)

        self._lock = threading.Lock()

        rospy.Subscriber('roomba_observations', OdometryArray, self._roomba_callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down roomba state estimator");

    def _roomba_callback(self, msg):
        with self._lock:
            roomba_odoms_msg = OdometryArray()
            roomba_states_msg = RoombaStateStampedArray()

            for roomba_pose in msg.data:
                roomba_id = roomba_pose.child_frame_id

                # check to see if we have seen this roomba yet
                if not roomba_id in self._roomba_dict:
                    roomba = Roomba(roomba_id)
                    self._roomba_dict[roomba_id] = roomba
                else:
                    roomba = self._roomba_dict.get(roomba_id)

                roomba_odoms_msg.data.append(roomba.update_odom(roomba_pose))

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
