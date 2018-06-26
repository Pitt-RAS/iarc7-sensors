#!/usr/bin/env python2
import rospy
import tf2_ros
import threading
from iarc7_msgs.msg import OdometryArray
from geometry_msgs.msg import TransformStamped

class RoombaTransformFilter(object):
    def __init__(self):
        rospy.init_node('roomba_transform_filter')

        self._lock = threading.Lock()

        self._transform_broadcaster = tf2_ros.TransformBroadcaster()

        self._last_published_stamp = rospy.Time(0)

        self._odometry_sub = rospy.Subscriber(
                'roombas',
                OdometryArray,
                self._callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down roomba_transform_filter");

    def _construct_transform(self, odom):
        transform_msg = TransformStamped()

        transform_msg.header.stamp = odom.header.stamp
        transform_msg.header.frame_id = 'map'
        transform_msg.child_frame_id = odom.child_frame_id

        transform_msg.transform.rotation = odom.pose.pose.orientation

        transform_msg.transform.translation.x = odom.pose.pose.position.x
        transform_msg.transform.translation.y = odom.pose.pose.position.y
        transform_msg.transform.translation.z = 0

        return transform_msg

    def _callback(self, msg):
        with self._lock:
            if len(msg.data) and self._last_published_stamp >= msg.data[0].header.stamp:
                return
            for odom in msg.data:
                transform_msg = self._construct_transform(odom)
                self._transform_broadcaster.sendTransform(transform_msg)

if __name__ == '__main__':
    RoombaTransformFilter()
