#!/usr/bin/env python

from __future__ import division

import math
import numpy as np
import rospy
import shapely
import threading

from iarc7_msgs.msg import RoombaDetectionFrame
from iarc7_msgs.msg import OdometryArray
from nav_msgs.msg import Odometry

from iarc7_sensors.roomba_filter.single_roomba_filter import SingleRoombaFilter
from ros_utils.make_safe_callback import make_safe_callback

class RoombaFilter(object):
    def __init__(self):
        self._lock = threading.Lock()
        with self._lock:
            rospy.Subscriber('/detected_roombas',
                             RoombaDetectionFrame,
                             make_safe_callback(self.callback))
            self._pub = rospy.Publisher('/roombas', OdometryArray, queue_size=10)
            self._debug_pub = rospy.Publisher('/single_roomba_odom', Odometry, queue_size=30)

            # TODO: this won't work for multiple cameras yet

            # Each filter is a dict, with fields
            # 'filter': SingleRoombaFilter object
            # 'last_seen': last time that roomba was seen
            self._filters = []
            self._world_fixed_frame = None

            self._last_msg_time = rospy.Time(0)

            self._queue = []
            self._last_fusion_time = rospy.Time(0)

    def callback(self, msg):
        with self._lock:
            if msg.header.stamp < self._last_fusion_time:
                rospy.logerr(
                        'Roomba filter received message older than fusion horizon, skipping')
                return
            self._queue.append(msg)
            self._queue.sort(key=lambda msg: msg.header.stamp)

    def _process_msg(self, msg):
        time = msg.header.stamp
        assert time >= self._last_msg_time
        dt = (time - self._last_msg_time).to_sec()
        self._last_msg_time = time

        if self._world_fixed_frame is None:
            self._world_fixed_frame = msg.header.frame_id
        elif msg.header.frame_id != self._world_fixed_frame:
            raise Exception(('Message in frame "{}" passed to roomba'
                          + 'filter, frame "{}" expected')
                           .format(msg.header.frame_id,
                                   self._world_fixed_frame))

        if msg.camera_id == 'bottom_camera':
            camera_pos_confidence = 1.0
            camera_neg_confidence = 1.0
        else:
            camera_pos_confidence = 0.8
            camera_neg_confidence = 0.2

        self._prune_filters(time)

        seen_filters = []
        for roomba in msg.roombas:
            f = self._get_filter_for_roomba(time, roomba)
            f['filter'].update(time, roomba)
            f['last_time'] = time
            seen_filters.append(f)

        LEAKAGE_RATE = 5
        self._update_scores(time,
                            dt,
                            seen_filters,
                            camera_pos_confidence,
                            camera_neg_confidence,
                            LEAKAGE_RATE,
                            msg.detection_region)

        self._publish(time)

    @staticmethod
    def mahalanobis_distance(pos, center, covariance):
        diff = pos - center
        distance = diff.T.dot(np.linalg.inv(covariance)).dot(diff)[0,0]
        return distance

    @staticmethod
    def point_in_poly(pos, poly):
        point = shapely.geometry.Point(pos.x, pos.y)
        polygon = shapely.geometry.Polygon([
            (point.x, point.y) for point in poly.points
            ])
        return polygon.contains(point)

    def run(self):
        FUSION_HORIZON = rospy.Duration(0.1)
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
        Decrement counters for roomba we should have seen
        '''
        # TODO: implement this
        pass

    def _get_filter_for_roomba(self, time, roomba):
        '''
        Find the best filter for the given roomba observation at the given time
        If one doesn't exist, create it
        '''
        MATCH_MAHALANOBIS_THRESHOLD = 50
        ROOMBA_RADIUS = 0.15
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
            [roomba.pose.x],
            [roomba.pose.y]], dtype=float)
        pos_cov = np.array([
            [roomba.position_covariance[0], roomba.position_covariance[1]],
            [roomba.position_covariance[2], roomba.position_covariance[3]]],
            dtype=float)

        for f in self._filters:
            odom = f['filter'].get_state(time)
            filter_pos = np.array([
                [odom.pose.pose.position.x],
                [odom.pose.pose.position.y]], dtype=float)
            filter_cov = np.array([
                [odom.pose.covariance[0], odom.pose.covariance[1]],
                [odom.pose.covariance[6], odom.pose.covariance[7]]], dtype=float)
            mahalanobis_distance = RoombaFilter.mahalanobis_distance(
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
            or best_distance < ROOMBA_RADIUS):
            if second_best_mahalanobis - best_mahalanobis < 0.3:
                rospy.logerr(('Roomba filter not sure which filter to fuse'
                        + ' measurement\n%s\nat time %.3f.  Best is at location'
                        + ' \n%s\n with covariance \n%s\n (mahalanobis %f), second best'
                        + ' is at location \n%s\n with covariance \n%s\n (mahalanobis %f)')
                        % (roomba, time.to_sec(), best_filter_pos, best_filter_cov,
                            best_mahalanobis, second_best_filter_pos,
                            second_best_filter_cov, second_best_mahalanobis))
            return best_filter

        rospy.loginfo('New roobma seen')
        rospy.loginfo('Best mahalanobis: %f, best distance %s'%(best_mahalanobis, best_distance))
        new_filter = {
                'filter': SingleRoombaFilter(self._world_fixed_frame),
                'last_time': None,
                'score': 0.0
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
        STDDEV_THRESHOLD = 0.5

        # Min score
        SCORE_THRESHOLD = -0.5

        for i in range(len(self._filters)-1, -1, -1):
            f = self._filters[i]
            if time - f['last_time'] > TIME_THRESHOLD:
                rospy.loginfo('Pruning roomba based on time threshold')
                del self._filters[i]
                continue

            if f['score'] < SCORE_THRESHOLD:
                rospy.loginfo('Pruning roomba for score')
                del self._filters[i]
                continue

            odom = f['filter'].get_state(time)
            cov = np.array([
                [odom.pose.covariance[0], odom.pose.covariance[1]],
                [odom.pose.covariance[6], odom.pose.covariance[7]]], dtype=float)
            max_stddev = math.sqrt(np.max(np.linalg.eigvalsh(cov)))

            if max_stddev > STDDEV_THRESHOLD:
                rospy.loginfo('Pruning roomba based on covariance threshold')
                del self._filters[i]
                continue

    def _publish(self, time):
        SCORE_PUBLISH_THRESHOLD = 0.99

        out_msg = OdometryArray()
        for f in self._filters:
            if f['score'] < SCORE_PUBLISH_THRESHOLD:
                continue
            odom = f['filter'].get_state(time)
            self._debug_pub.publish(odom)
            out_msg.data.append(odom)
        self._pub.publish(out_msg)

    def _update_scores(self,
                       time,
                       dt,
                       seen_filters,
                       camera_pos_confidence,
                       camera_neg_confidence,
                       leakage_rate,
                       camera_poly):
        for f in self._filters:
            if f in seen_filters:
                f['score'] += camera_pos_confidence
            else:
                odom = f['filter'].get_state(time)
                pos = odom.pose.pose.position
                if RoombaFilter.point_in_poly(pos, camera_poly):
                    f['score'] -= camera_neg_confidence
                else:
                    f['score'] -= leakage_rate * dt

            MAX_SCORE = 10.
            if f['score'] > MAX_SCORE:
                f['score'] = MAX_SCORE

if __name__ == '__main__':
    rospy.init_node('roomba_filter')

    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time(0):
        rate.sleep()

    roomba_filter = RoombaFilter()
    roomba_filter.run()
