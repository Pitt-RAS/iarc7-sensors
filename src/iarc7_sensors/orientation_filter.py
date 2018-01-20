#!/usr/bin/env python2

from iarc7_safety.SafetyClient import SafetyClient
import math
import rospy
import tf.transformations
import tf2_ros
import threading

from geometry_msgs.msg import TransformStamped
from iarc7_msgs.msg import (Float64Stamped,
                            OrientationAnglesStamped)

class OrientationFilter(object):
    def __init__(self):
        rospy.init_node('orientation_filter')

        self._lock = threading.Lock()

        self._orientation_sub = rospy.Subscriber(
                'fc_orientation',
                OrientationAnglesStamped,
                lambda msg: self._callback(OrientationAnglesStamped, msg))

        # This topic is assumed to have rotation around +z
        # (angle from +x with +y being +pi/2)
        self._line_yaw_sub = rospy.Subscriber(
                'line_yaw',
                Float64Stamped,
                lambda msg: self._callback(Float64Stamped, msg))

        self._debug_orientation_pub = rospy.Publisher(
                'orientation_filter/debug_orientation',
                OrientationAnglesStamped,
                queue_size=10)

        self._line_weight = rospy.get_param('~line_weight')
        self._message_queue_length = rospy.get_param('~message_queue_length')

        initial_msg = Float64Stamped()
        initial_msg.header.stamp = rospy.Time()
        initial_msg.data = rospy.get_param('~initial_yaw', float('NaN'))

        # This queue contains pairs, sorted by timestamp, oldest at index 0
        #
        # First item of each pair is a 3-tuple containing:
        #     yaw (rotation around +z)
        #     pitch (rotation around +y')
        #     roll (rotation around +x'')
        # Second item of each pair is the message itself
        #
        # The yaw in the 3-tuple is the current value of the filter at that
        # point, the pitch and roll are too except that they're simply
        # propogated forward from the last OrientationAnglesStamped message
        self._queue = [([initial_msg.data, float('NaN'), float('NaN')],
                        initial_msg)]

        self._transform_broadcaster = tf2_ros.TransformBroadcaster()

        self._last_published_stamp = rospy.Time(0)

    def _publish_transform(self, r, p, y, time):
        transform_msg = TransformStamped()

        transform_msg.header.stamp = time
        transform_msg.header.frame_id = 'level_quad'
        transform_msg.child_frame_id = 'quad'

        quaternion = tf.transformations.quaternion_from_euler(r, p, y)
        transform_msg.transform.rotation.x = quaternion[0]
        transform_msg.transform.rotation.y = quaternion[1]
        transform_msg.transform.rotation.z = quaternion[2]
        transform_msg.transform.rotation.w = quaternion[3]

        self._transform_broadcaster.sendTransform(transform_msg)

        transform_msg.header.stamp = time
        transform_msg.header.frame_id = 'level_quad'
        transform_msg.child_frame_id = 'heading_quad'

        quaternion = tf.transformations.quaternion_from_euler(0, 0, y)
        transform_msg.transform.rotation.x = quaternion[0]
        transform_msg.transform.rotation.y = quaternion[1]
        transform_msg.transform.rotation.z = quaternion[2]
        transform_msg.transform.rotation.w = quaternion[3]

        self._transform_broadcaster.sendTransform(transform_msg)

        orientation_message = OrientationAnglesStamped()
        orientation_message.header.stamp = time
        orientation_message.data.pitch = p
        orientation_message.data.roll = r
        orientation_message.data.yaw = y

        self._debug_orientation_pub.publish(orientation_message)

    def _get_last_index(self, klass, time=None):
        '''
        Return the index of the last message of type `klass` (or the
        last message of type `klass` before `time` if `time` is
        specified)

        Returns `-1` if there are no messages of type `klass` in the
        queue
        '''
        for i in xrange(len(self._queue)-1, -1, -1):
            if (type(self._queue[i][1]) == klass
                 and (time is None or self._queue[i][1].header.stamp < time)):
                return i
        return -1

    def _callback(self, klass, msg):
        with self._lock:
            last_i = self._get_last_index(klass)
            if (last_i != -1
                and msg.header.stamp <= self._queue[last_i][1].header.stamp):

                # This message isn't newer than the last one of type klass
                rospy.logwarn(
                    ('Ignoring message of type {} with timestamp {} older than '
                   + 'previous message with timestamp {}')
                        .format(
                            klass,
                            msg.header.stamp,
                            self._queue[last_i][1].header.stamp))
                return

            for i in xrange(len(self._queue) - 1, -1, -1):
                if self._queue[i][1].header.stamp < msg.header.stamp:
                    index = i+1
                    break
            else:
                index = 0

            data = [float('NaN')] * 3
            try:
                data[1] = -msg.data.pitch
                data[2] = msg.data.roll
            except AttributeError:
                # If this message type doesn't have pitch and roll, skip it
                pass

            self._queue.insert(index, (data, msg))
            self._reprocess(index)

            if (not math.isnan(self._queue[-1][0][0])
                and not math.isnan(self._queue[-1][0][1])
                and not math.isnan(self._queue[-1][0][2])):

                # Make sure new timestamp is at least 1us newer than the
                # last one. Using 1ns was dangerous and caused the tf library
                # to create transforms with NaN values
                new_stamp = max(self._last_published_stamp + rospy.Duration(0, 1000),
                                self._queue[-1][1].header.stamp)
                self._publish_transform(*reversed(self._queue[-1][0]), time=new_stamp)
                self._last_published_stamp = new_stamp

            while len(self._queue) > self._message_queue_length:
                self._queue.pop(0)

    def _reprocess(self, index):
        for i in xrange(index, len(self._queue)):
            last_yaw = self._queue[i-1][0][0]
            msg = self._queue[i][1]
            last_orientation_i = self._get_last_index(OrientationAnglesStamped,
                                                      msg.header.stamp)

            if type(msg) == OrientationAnglesStamped:

                if last_orientation_i == -1:
                    dyaw = 0
                else:
                    dyaw = -(msg.data.yaw
                           - self._queue[last_orientation_i][1].data.yaw)

                if dyaw > math.pi:
                    dyaw -= 2*math.pi
                elif dyaw < -math.pi:
                    dyaw += 2*math.pi

                self._queue[i][0][0] = last_yaw + dyaw
            elif type(msg) == Float64Stamped:
                new_yaw = msg.data
                if new_yaw - last_yaw > math.pi:
                    new_yaw -= 2*math.pi
                elif new_yaw - last_yaw < -math.pi:
                    new_yaw += 2*math.pi

                # Skip the filtering step if we don't already have a valid
                # measurement
                if math.isnan(last_yaw):
                    self._queue[i][0][0] = new_yaw
                else:
                    self._queue[i][0][0] = (self._line_weight     * new_yaw
                                          + (1-self._line_weight) * last_yaw)

                if last_orientation_i != -1:
                    self._queue[i][0][1] = self._queue[last_orientation_i][0][1]
                    self._queue[i][0][2] = self._queue[last_orientation_i][0][2]
            else:
                assert False

            self._queue[i][0][0] += 2*math.pi
            self._queue[i][0][0] %= 2*math.pi

    def run(self):
        self._safety_client = SafetyClient('orientation_filter')
        assert self._safety_client.form_bond()

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if (self._safety_client.is_fatal_active()
             or self._safety_client.is_safety_active()):
                break
            rate.sleep()

if __name__ == '__main__':
    orientation_filter = OrientationFilter()
    orientation_filter.run()
