#!/usr/bin/env python

from __future__ import division

import math
import numpy as np
import rospy
import threading
import uuid

from iarc7_msgs.msg import ObstacleArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from iarc7_sensors.roomba_filter.single_obstacle_filter import SingleObstacleFilter
from ros_utils.make_safe_callback import make_safe_callback

class ObstacleFilter(object):
    def __init__(self):
        self._lock = threading.Lock()
        with self._lock:
            rospy.Subscriber('/detected_obstacles',
                             ObstacleArray,
                             make_safe_callback(self.callback))
            self._pub = rospy.Publisher('/obstacles', ObstacleArray, queue_size=10)
            self._marker_pub = rospy.Publisher(
                    '/obstacle_markers', MarkerArray, queue_size=10)

            # TODO: this won't work for multiple cameras yet

            # Each filter is a dict, with fields
            # 'filter': SingleObstacleFilter object
            # 'last_seen': last time that obstacle was seen
            self._filters = []
            self._world_fixed_frame = None

            self._last_msg_time = rospy.Time(0)

            self._queue = []
            self._last_fusion_time = rospy.Time(0)

    def callback(self, msg):
        with self._lock:
            if msg.header.stamp < self._last_fusion_time:
                rospy.logerr(
                        'Obstacle filter received message older than fusion horizon, skipping')
                return
            self._queue.append(msg)
            self._queue.sort(key=lambda msg: msg.header.stamp)

    def _process_msg(self, msg):
        time = msg.header.stamp
        assert time >= self._last_msg_time
        self._last_msg_time = time

        if self._world_fixed_frame is None:
            self._world_fixed_frame = msg.header.frame_id
        elif msg.header.frame_id != self._world_fixed_frame:
            raise Exception(('Message in frame "{}" passed to obstacle'
                          + 'filter, frame "{}" expected')
                           .format(msg.header.frame_id,
                                   self._world_fixed_frame))

        self._decrement_counters(msg)
        self._prune_filters(time)

        for obstacle in msg.obstacles:
            f = self._get_filter_for_obstacle(time, obstacle)
            f['filter'].update(time, obstacle)
            f['last_time'] = time

        self._publish(time)

    @staticmethod
    def mahalanobis_distance(pos, center, covariance):
        diff = pos - center
        distance = diff.T.dot(np.linalg.inv(covariance)).dot(diff)[0,0]
        return distance

    def run(self):
        FUSION_HORIZON = rospy.Duration(0.07)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            curr_time = rospy.Time.now()
            fusion_time = curr_time - FUSION_HORIZON
            with self._lock:
                for i, msg in enumerate(self._queue):
                    if msg.header.stamp > fusion_time:
                        del self._queue[:i]
                        break
                    self._process_msg(msg)
                else:
                    # We didn't hit any messages newer than the fusion time, so
                    # clear the whole queue
                    self._queue = []
                self._last_fusion_time = fusion_time
            rate.sleep()

    def _decrement_counters(self, msg):
        '''
        Decrement counters for obstacle we should have seen
        '''
        # TODO: implement this
        pass

    def _get_filter_for_obstacle(self, time, obstacle):
        '''
        Find the best filter for the given obstacle observation at the given time
        If one doesn't exist, create it
        '''
        MATCH_MAHALANOBIS_THRESHOLD = 100
        OBSTACLE_RADIUS = 0.15
        MATCH_DISTANCE_THRESHOLD = 1.0
        best_filter = None
        best_filter_pos = None
        best_filter_cov = None
        best_mahalanobis = float('Inf')
        best_distance = float('Inf')
        second_best_filter = None
        second_best_filter_pos = None
        second_best_filter_cov = None
        second_best_mahalanobis = float('Inf')
        second_best_distance = float('Inf')

        pos = np.array([
            [obstacle.odom.pose.pose.position.x],
            [obstacle.odom.pose.pose.position.y]], dtype=float)
        pos_cov = np.array([
            [obstacle.odom.pose.covariance[0], obstacle.odom.pose.covariance[1]],
            [obstacle.odom.pose.covariance[6], obstacle.odom.pose.covariance[7]]],
            dtype=float)

        for f in self._filters:
            odom = f['filter'].get_state(time).odom
            filter_pos = np.array([
                [odom.pose.pose.position.x],
                [odom.pose.pose.position.y]], dtype=float)
            filter_cov = np.array([
                [odom.pose.covariance[0], odom.pose.covariance[1]],
                [odom.pose.covariance[6], odom.pose.covariance[7]]], dtype=float)
            mahalanobis_distance = ObstacleFilter.mahalanobis_distance(
                    pos, filter_pos, filter_cov + pos_cov)
            distance = np.linalg.norm(pos - filter_pos)

            if mahalanobis_distance < best_mahalanobis:
                second_best_filter = best_filter
                second_best_filter_pos = best_filter_pos
                second_best_filter_cov = best_filter_cov
                second_best_mahalanobis = best_mahalanobis
                second_best_distance = best_distance
                best_filter = f
                best_filter_pos = filter_pos
                best_filter_cov = filter_cov
                best_mahalanobis = mahalanobis_distance
                best_distance = distance

        if ((best_mahalanobis < MATCH_MAHALANOBIS_THRESHOLD
                and best_distance < MATCH_DISTANCE_THRESHOLD)
            or best_distance < OBSTACLE_RADIUS):
            if second_best_mahalanobis - best_mahalanobis < 0.3:
                rospy.logerr(('Obstacle filter not sure which filter to fuse'
                        + ' measurement\n%s\nat time %.3f.  Best is at location'
                        + ' \n%s\n with covariance \n%s\n (mahalanobis %f), second best'
                        + ' is at location \n%s\n with covariance \n%s\n (mahalanobis %f)')
                        % (obstacle, time.to_sec(), best_filter_pos, best_filter_cov,
                            best_mahalanobis, second_best_filter_pos,
                            second_best_filter_cov, second_best_mahalanobis))
            return best_filter

        rospy.loginfo('New obstacle seen')
        rospy.loginfo('Best mahalanobis: %f, best distance %s'%(best_mahalanobis, best_distance))
        new_filter = {
                'filter': SingleObstacleFilter(self._world_fixed_frame),
                'last_time': None
            }
        self._filters.append(new_filter)
        return new_filter

    def _prune_filters(self, time):
        '''
        Throw out filters that don't meet criteria

        :param time: current time
        '''
        # Max time without sighting
        TIME_THRESHOLD = rospy.Duration(5.0)

        # Max position uncertainty
        STDDEV_THRESHOLD = 2.0

        for i in range(len(self._filters)-1, -1, -1):
            f = self._filters[i]
            if time - f['last_time'] > TIME_THRESHOLD:
                rospy.loginfo('Pruning obstacle based on time threshold')
                del self._filters[i]
                continue

            odom = f['filter'].get_state(time).odom
            cov = np.array([
                [odom.pose.covariance[0], odom.pose.covariance[1]],
                [odom.pose.covariance[6], odom.pose.covariance[7]]], dtype=float)
            max_stddev = math.sqrt(np.max(np.linalg.eigvalsh(cov)))

            if max_stddev > STDDEV_THRESHOLD:
                rospy.loginfo('Pruning obstacle based on covariance threshold')
                del self._filters[i]
                continue

    def _publish(self, time):
        assert time != rospy.Time(0)

        out_msg = ObstacleArray()
        out_msg.header.stamp = time
        out_msg.header.frame_id = 'map'

        marker_array = MarkerArray()

        for f in self._filters:
            obstacle = f['filter'].get_state(time)
            out_msg.obstacles.append(obstacle)

            marker = Marker()
            marker.header.stamp = time
            marker.header.frame_id = 'map'

            marker.ns = 'obstacles'

            marker.id = uuid.UUID('{'+obstacle.odom.child_frame_id[len('obstacle-'):-len('/base_link')]+'}').int % (2**31)

            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0

            marker.scale.x = obstacle.pipe_radius
            marker.scale.y = obstacle.pipe_radius
            marker.scale.z = obstacle.pipe_height

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime = rospy.Duration(0.2)

            marker.pose.position.x = obstacle.odom.pose.pose.position.x
            marker.pose.position.y = obstacle.odom.pose.pose.position.y
            marker.pose.position.z = obstacle.pipe_height / 2.
            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)
        self._pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('obstacle_filter')

    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time(0):
        rate.sleep()

    obstacle_filter = ObstacleFilter()
    obstacle_filter.run()
