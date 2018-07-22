#!/usr/bin/env python

from __future__ import division

import rospy
import threading

from iarc7_msgs.msg import RoombaDetectionFrame
from iarc7_msgs.msg import OdometryArray
from nav_msgs.msg import Odometry

from iarc7_sensors.roomba_filter.single_roomba_filter import SingleRoombaFilter
from ros_utils.make_safe_callback import make_safe_callback

class SimpleRoombaFilter(object):
    def __init__(self):
        self._lock = threading.Lock()
        with self._lock:
            rospy.Subscriber('/detected_roombas',
                             RoombaDetectionFrame,
                             make_safe_callback(self.callback))
            self._pub = rospy.Publisher('/roombas', OdometryArray, queue_size=10)
            self._debug_pub = rospy.Publisher('/single_roomba_odom', Odometry, queue_size=10)
            self._timer = rospy.Timer(rospy.Duration(0.1),
                                      make_safe_callback(self._timer_callback))
            self._filter = None
            self._world_fixed_frame = None

            self._last_msg_time = rospy.Time(0)

    def callback(self, msg):
        with self._lock:
            assert msg.header.stamp >= self._last_msg_time
            self._last_msg_time = msg.header.stamp

            if self._world_fixed_frame is None:
                self._world_fixed_frame = msg.header.frame_id
            elif msg.header.frame_id != self._world_fixed_frame:
                raise Exception('Message in frame {} passed to roomba'
                               + 'filter, frame {} expected'
                               .format(msg.header.frame_id,
                                       self._world_fixed_frame))

            if not msg.roombas and self._filter is None:
                # TODO: make vision always publish on frame, even below
                # detection height
                return

            if not msg.roombas:
                self._publish(msg.header.stamp)
                return

            roomba = msg.roombas[0]
            if self._filter is None:
                self._filter = SingleRoombaFilter(self._world_fixed_frame)

            self._filter.update(msg.header.stamp, roomba)
            self._publish(msg.header.stamp)

    def _publish(self, time):
        out_msg = OdometryArray()
        odom = self._filter.get_state(time)
        out_msg.data.append(odom)
        self._pub.publish(out_msg)
        self._debug_pub.publish(odom)

    def _publish_empty(self):
        out_msg = OdometryArray()
        self._pub.publish(out_msg)

    def _timer_callback(self, timer_event):
        with self._lock:
            if self._filter is not None:
                self._timer.shutdown()
            else:
                self._publish_empty()

if __name__ == '__main__':
    rospy.init_node('simple_roomba_filter')
    simple_roomba_filter = SimpleRoombaFilter()
    rospy.spin()
