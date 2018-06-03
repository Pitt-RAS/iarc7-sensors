#!/usr/bin/env python

import rospy
import tf.transformations

import numpy as np
import threading

from iarc7_msgs.msg import RoombaDetectionFrame
from iarc7_msgs.msg import OdometryArray
from nav_msgs.msg import Odometry

from iarc7_sensors.roomba_filter.kalman_filter_2d import KalmanFilter2d

class SimpleRoobmaFilter(object):
    def __init__(self):
        self._lock = threading.Lock()
        with self._lock:
            rospy.Subscriber('/detected_roombas', RoombaDetectionFrame, self.callback)
            self._pub = rospy.Publisher('/roombas', OdometryArray, queue_size=10)
            self._debug_pub = rospy.Publisher('/single_roomba_odom', Odometry, queue_size=10)
            self._timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)
            self._kf_2d = KalmanFilter2d()

    def callback(self, msg):
        with self._lock:
            if not msg.roombas and not self._kf_2d.initialized():
                return

            if not msg.roombas:
                self._kf_2d.predict(msg.header.stamp)
                self._publish()
                return

            roomba = msg.roombas[0]
            if not self._kf_2d.initialized():
                self._kf_2d.set_state(
                        msg.header.stamp,
                        np.array([[roomba.pose.x],
                                  [roomba.pose.y],
                                  [0],
                                  [0]], dtype=float))
                return

            self._kf_2d.update(
                    msg.header.stamp,
                    np.array([roomba.pose.x, roomba.pose.y]))
            self._publish()

    def _publish(self):
        state = self._kf_2d.get_state()

        out_msg = OdometryArray()
        odom = Odometry()
        odom.header.stamp = self._last_time
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'roomba0/base_link'
        odom.pose.pose.position.x = state[0]
        odom.pose.pose.position.y = state[1]

        yaw = np.arctan2(state[3], state[2])
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

    def _publish_empty(self):
        out_msg = OdometryArray()
        self._pub.publish(out_msg)

    def _timer_callback(self):
        with self._lock:
            if self._kf_2d.initialized():
                self._timer.shutdown()
            else:
                self._publish_empty()

if __name__ == '__main__':
    rospy.init_node('simple_roomba_filter')
    simple_roomba_filter = SimpleRoobmaFilter()
    rospy.spin()
