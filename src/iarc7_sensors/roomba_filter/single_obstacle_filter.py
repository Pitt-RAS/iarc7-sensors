from __future__ import division

import math
import numpy as np
import rospy
import tf.transformations
import threading
import uuid

from iarc7_msgs.msg import Obstacle, ObstacleArray
from nav_msgs.msg import Odometry

from iarc7_sensors.roomba_filter.kalman_filter_2d import KalmanFilter2d

class SingleObstacleFilter(object):
    def __init__(self, world_fixed_frame):
        self._lock = threading.Lock()
        with self._lock:
            self._id = uuid.uuid4()
            self._world_fixed_frame = world_fixed_frame
            self._frame = 'obstacle-{}/base_link'.format(self._id)
            self._kf = KalmanFilter2d()

            self._curr_base_radius = 0.0
            self._curr_base_height = 0.0
            self._curr_pipe_radius = 0.0
            self._curr_pipe_height = 0.0

    def _predict(self, time):
        self._kf.predict(time)

    def predict(self, time):
        with self._lock:
            self._predict(time)

    def update(self, time, obstacle_detection_msg):
        with self._lock:
            if self._kf.initialized():
                self._predict(time)

            state = np.array([obstacle_detection_msg.odom.pose.pose.position.x,
                              obstacle_detection_msg.odom.pose.pose.position.y],
                              dtype=float)

            covariance_msg = obstacle_detection_msg.odom.pose.covariance
            covariance = np.array([
                [covariance_msg[0], covariance_msg[1]],
                [covariance_msg[6], covariance_msg[7]]], dtype=float)

            self._kf.update(time, state, covariance)

            self._curr_base_radius = max(self._curr_base_radius,
                                         obstacle_detection_msg.base_radius)
            self._curr_base_height = max(self._curr_base_height,
                                         obstacle_detection_msg.base_height)
            self._curr_pipe_radius = max(self._curr_pipe_radius,
                                         obstacle_detection_msg.pipe_radius)
            self._curr_pipe_height = max(self._curr_pipe_height,
                                         obstacle_detection_msg.pipe_height)

    def get_state(self, time):
        with self._lock:
            self._predict(time)

            _, state, cov = self._kf.get_state()
            pos = state[:2,0]
            pos_cov = cov[:2,:2]
            vel = state[2:,0]
            theta = math.atan2(vel[1], vel[0])
            z_rate = 0

            obstacle = Obstacle()
            obstacle.header.stamp = time
            obstacle.header.frame_id = self._world_fixed_frame

            obstacle.base_height = self._curr_base_height
            obstacle.base_radius = self._curr_base_radius
            obstacle.pipe_height = self._curr_pipe_height
            obstacle.pipe_radius = self._curr_pipe_radius

            odom = Odometry()
            odom.header.stamp = time
            odom.header.frame_id = self._world_fixed_frame
            odom.child_frame_id = self._frame
            odom.pose.pose.position.x = pos[0]
            odom.pose.pose.position.y = pos[1]
            odom.pose.covariance[0] = pos_cov[0,0]
            odom.pose.covariance[1] = pos_cov[0,1]
            odom.pose.covariance[6] = pos_cov[0,1]
            odom.pose.covariance[7] = pos_cov[1,1]

            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            odom.pose.pose.orientation.x = quat[0]
            odom.pose.pose.orientation.y = quat[1]
            odom.pose.pose.orientation.z = quat[2]
            odom.pose.pose.orientation.w = quat[3]

            odom.twist.twist.linear.x = vel[0]
            odom.twist.twist.linear.y = vel[1]
            odom.twist.twist.angular.z = z_rate

            obstacle.odom = odom

            return obstacle
