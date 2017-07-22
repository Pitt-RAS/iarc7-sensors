#!/usr/bin/env python2

from iarc7_safety.SafetyClient import SafetyClient
import rospy
import threading

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

class VelocityFilter(object):
    def __init__(self):
        rospy.init_node('velocity_filter')

        self._lock = threading.Lock()

        self._optical_flow_sub = rospy.Subscriber(
                '/optical_flow_estimator/twist',
                TwistWithCovarianceStamped,
                lambda msg: self._callback('optical', msg))

        # This topic is assumed to have rotation around +z
        # (angle from +x with +y being +pi/2)
        self._kalman_filter_sub = rospy.Subscriber(
                '/odometry/filtered',
                Odometry,
                lambda msg: self._callback('kalman', msg))

        self._kalman_weight = rospy.get_param('~kalman_weight')
        self._message_queue_length = rospy.get_param('~message_queue_length')

        initial_msg = TwistWithCovarianceStamped()
        initial_msg.header.stamp = rospy.Time()

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
        self._queue = [([0.0, 0.0],
                        initial_msg, 'optical')]

        self._vel_pub = rospy.Publisher('double_filtered_vel', TwistWithCovarianceStamped, queue_size=10)

        self._last_published_stamp = rospy.Time(0)

    def _publish_vel(self, x, y, time):
        msg = TwistWithCovarianceStamped()

        msg.header.stamp = time
        msg.header.frame_id = 'map'

        msg.twist.twist.linear.x = x
        msg.twist.twist.linear.y = y

        self._vel_pub.publish(msg)

    def _get_last_index(self, klass, time=None):
        '''
        Return the index of the last message of type `klass` (or the
        last message of type `klass` before `time` if `time` is
        specified)

        Returns `-1` if there are no messages of type `klass` in the
        queue
        '''
        for i in xrange(len(self._queue)-1, -1, -1):
            if (self._queue[i][2] == klass
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
                rospy.logerr('Skipping message older than everything in the queue')
                return

            self._queue.insert(index, ([float('NaN'), float('NaN')], msg, klass))
            self._reprocess(index)

            # Make sure new timestamp is at least 1ns newer than the
            # last one
            new_stamp = max(self._last_published_stamp + rospy.Duration(0, 1),
                            self._queue[-1][1].header.stamp)
            self._publish_vel(*self._queue[-1][0], time=new_stamp)
            self._last_published_stamp = new_stamp

            while len(self._queue) > self._message_queue_length:
                self._queue.pop(0)

    def _reprocess(self, index):
        for i in xrange(index, len(self._queue)):
            msg = self._queue[i][1]
            klass = self._queue[i][2]
            last_differential_i = self._get_last_index('kalman',
                                                       msg.header.stamp)

            if klass == 'kalman':
                if last_differential_i == -1:
                    dx = 0.0
                    dy = 0.0
                else:
                    dx = -(msg.twist.twist.linear.x
                           - self._queue[last_differential_i][1].twist.twist.linear.x)
                    dy = -(msg.twist.twist.linear.y
                           - self._queue[last_differential_i][1].twist.twist.linear.y)

                self._queue[i][0][0] = self._queue[i-1][0][0] + dx
                self._queue[i][0][1] = self._queue[i-1][0][1] + dy

            elif klass == 'optical':
                self._queue[i][0][0] = ((1-self._kalman_weight)     * msg.twist.linear.x
                                      + self._kalman_weight * self._queue[i-1][0][0])
                self._queue[i][0][1] = ((1-self._kalman_weight)     * msg.twist.linear.y
                                      + self._kalman_weight * self._queue[i-1][0][1])
            else:
                assert False

    def run(self):
        self._safety_client = SafetyClient('velocity_filter')
        assert self._safety_client.form_bond()

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if (self._safety_client.is_fatal_active()
             or self._safety_client.is_safety_active()):
                break
            rate.sleep()

if __name__ == '__main__':
    velocity_filter = VelocityFilter()
    velocity_filter.run()
