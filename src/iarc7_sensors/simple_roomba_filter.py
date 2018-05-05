#!/usr/bin/env python

import rospy
import tf.transformations

import numpy as np
import threading

from iarc7_msgs.msg import RoombaDetectionFrame
from iarc7_msgs.msg import OdometryArray
from nav_msgs.msg import Odometry

class SimpleRoobmaFilter(object):
    '''
    State [x, y, vx, vy]
    '''

    def __init__(self):
        self._lock = threading.Lock()
        with self._lock:
            rospy.Subscriber('/detected_roombas', RoombaDetectionFrame, self.callback)
            self._pub = rospy.Publisher('/roombas', OdometryArray, queue_size=10)
            self._debug_pub = rospy.Publisher('/single_roomba_odom', Odometry, queue_size=10)
            self._last_time = None

            # Initial uncertainty
            self.P = np.array((
                (0.01, 0, 0, 0),
                (0, 0.01, 0, 0),
                (0, 0, 10, 0),
                (0, 0, 0, 10)))

            # Process noise
            self.Q = np.array((
                (0.05,  0,  0,  0 ),
                (0,  0.05,  0,  0 ),
                (0,  0,  3,  0 ),
                (0,  0,  0,  3 )), dtype=float)

            self.H = np.array((
                (1, 0, 0, 0),
                (0, 1, 0, 0)), dtype=float)

            # Measurement noise
            self.R = np.array((
                (0.05, 0),
                (0, 0.05)), dtype=float)

    def callback(self, msg):
        with self._lock:
            if not msg.roombas and self._last_time is None:
                return

            if not msg.roombas:
                dt = (msg.header.stamp - self._last_time).to_sec()

                if dt <= 0:
                    rospy.logerr(
                            'SimpleRoombaFilter received messages less than 0ns apart: %s %s',
                            self._last_time,
                            msg.header.stamp)
                    return

                F = np.array((
                    (1,  0,  dt, 0 ),
                    (0,  1,  0,  dt),
                    (0,  0,  1,  0 ),
                    (0,  0,  0,  1 )), dtype=float)
                Q = self.Q * dt**2

                self.s = F.dot(self.s)
                self.P = F.dot(self.P).dot(F.T) + Q

                self._last_time = msg.header.stamp
                self._publish()
                return

            roomba = msg.roombas[0]
            if self._last_time is None:
                self.s = np.array([[roomba.pose.x],
                                   [roomba.pose.y],
                                   [0],
                                   [0]], dtype=float)
                self._last_time = msg.header.stamp
                return

            dt = (msg.header.stamp - self._last_time).to_sec()

            if dt <= 0:
                rospy.logerr(
                        'SimpleRoombaFilter received messages less than 0ns apart: %s %s',
                        self._last_time,
                        msg.header.stamp)
                return

            F = np.array((
                (1,  0,  dt, 0 ),
                (0,  1,  0,  dt),
                (0,  0,  1,  0 ),
                (0,  0,  0,  1 )), dtype=float)
            Q = self.Q * dt**2
            H = self.H
            R = self.R

            # predict
            s_new = F.dot(self.s)
            P_new = F.dot(self.P).dot(F.T) + Q

            # correct
            z = np.array((
                (roomba.pose.x,),
                (roomba.pose.y,)), dtype=float)
            innov = z - H.dot(s_new)
            innov_cov = R + H.dot(P_new).dot(H.T)
            K = P_new.dot(H.T).dot(np.linalg.inv(innov_cov))
            s_new = s_new + K.dot(innov)
            P_new = (np.eye(4) - K.dot(H)).dot(P_new)

            self.s = s_new
            self.P = P_new

            self._last_time = msg.header.stamp
            self._publish()

    def _publish(self):
        out_msg = OdometryArray()
        odom = Odometry()
        odom.header.stamp = self._last_time
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'roomba0'
        odom.pose.pose.position.x = self.s[0]
        odom.pose.pose.position.y = self.s[1]

        yaw = np.arctan2(self.s[3,0], self.s[2,0])
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        vel = np.array((
            (0.3,),
            (0,)), dtype=float)
        rot = np.array((
            (np.cos(yaw), -np.sin(yaw)),
            (np.sin(yaw), np.cos(yaw))), dtype=float)
        vel = rot.dot(vel)
        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.y = vel[1]

        out_msg.data.append(odom)
        self._pub.publish(out_msg)
        self._debug_pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('simple_roomba_filter')
    simple_roomba_filter = SimpleRoobmaFilter()
    rospy.spin()
