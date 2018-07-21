from __future__ import division

from collections import deque
from enum import Enum
import math
import numpy as np
import rospy
import tf.transformations
import threading
import uuid

from nav_msgs.msg import Odometry

from iarc7_sensors.roomba_filter.kalman_filter_2d import KalmanFilter2d
from iarc7_sensors.roomba_filter.kalman_filter_xytheta import ExtendedKalmanFilter2d
from iarc7_sensors.roomba_filter.orientation_seeder import OrientationSeeder
from iarc7_sensors.roomba_filter.stop_detector import stop_detector

class SingleRoombaFilterState(Enum):
    NO_ORIENTATION = 0
    HAVE_ORIENTATION = 1
    HAVE_ORIENTATION_STOPPED = 2

class SingleRoombaFilter(object):
    FULL_ROTATION_TIME = rospy.Duration(2.150)
    SPEED = 0.33
    ROTATION_RATE = -math.pi / FULL_ROTATION_TIME.to_sec()

    def __init__(self, world_fixed_frame):
        self._lock = threading.Lock()
        with self._lock:
            self._id = uuid.uuid4()
            self._world_fixed_frame = world_fixed_frame
            self._frame = 'roomba-{}/base_link'.format(self._id)
            self._world_fixed_frame = world_fixed_frame
            self._state = SingleRoombaFilterState.NO_ORIENTATION
            self._kf = KalmanFilter2d()
            self._ekf = ExtendedKalmanFilter2d(SingleRoombaFilter.SPEED)
            self._orientation_seeder = OrientationSeeder()
            self._measurements = deque()
            self._rotation_stop_time = None

    def _predict(self, time):
        if self._state == SingleRoombaFilterState.NO_ORIENTATION:
            self._kf.predict(time)
        elif self._state == SingleRoombaFilterState.HAVE_ORIENTATION:
            self._ekf.predict(time)
            self._ekf.update_theta_dot(time, 0.0, 0.2)
        elif self._state == SingleRoombaFilterState.HAVE_ORIENTATION_STOPPED:
            if time >= self._rotation_stop_time:
                self._ekf.predict(self._rotation_stop_time)
                self._ekf.set_v(SingleRoombaFilter.SPEED, 0, self._rotation_stop_time)
                self._state = SingleRoombaFilterState.HAVE_ORIENTATION
                self._rotation_stop_time = None
                self._ekf.predict(time)
            else:
                self._ekf.predict(time)
        else:
            assert False

    def predict(self, time):
        with self._lock:
            self._predict(time)

    def update(self, time, roomba_detection_msg):
        with self._lock:
            if self._kf.initialized():
                self._predict(time)

            state = np.array([roomba_detection_msg.pose.x,
                              roomba_detection_msg.pose.y], dtype=float)

            covariance_msg = roomba_detection_msg.position_covariance
            covariance = np.array([
                [covariance_msg[0], covariance_msg[1]],
                [covariance_msg[2], covariance_msg[3]]], dtype=float)

            self._measurements.append((time, roomba_detection_msg))

            if self._state == SingleRoombaFilterState.NO_ORIENTATION:
                self._kf.update(time, state, covariance)
                if roomba_detection_msg.flip_certainty > 0.5:
                    self._orientation_seeder.add_orientation_estimate(
                            roomba_detection_msg.pose.theta,
                            roomba_detection_msg.box_uncertainty)

                orientation = None
                orientation_uncertainty = float('Inf')

                kf_orientation_estimate, kf_uncertainty = self._kf.orientation()
                if kf_orientation_estimate is not None:
                    orientation = kf_orientation_estimate
                    orientation_uncertainty = kf_uncertainty

                if self._orientation_seeder.converged():
                    seeder_orientation_estimate, seeder_uncertainty = \
                            self._orientation_seeder.get_estimate()
                    if seeder_uncertainty < orientation_uncertainty:
                        orientation = seeder_orientation_estimate
                        orientation_uncertainty = seeder_uncertainty

                if orientation is not None:
                    self._state = SingleRoombaFilterState.HAVE_ORIENTATION
                    measurements_iter = iter(self._measurements)
                    first_time, first_msg = next(measurements_iter)
                    covariance_msg = first_msg.position_covariance
                    covariance = np.array([
                        [covariance_msg[0], covariance_msg[1]],
                        [covariance_msg[2], covariance_msg[3]]], dtype=float)
                    self._ekf.update_with_theta(
                            first_time,
                            np.array([first_msg.pose.x, first_msg.pose.y],
                                     dtype=float),
                            covariance,
                            orientation,
                            orientation_uncertainty,
                            False)
                    for time, msg in measurements_iter:
                        state = np.array([msg.pose.x, msg.pose.y], dtype=float)
                        covariance_msg = msg.position_covariance
                        covariance = np.array([
                            [covariance_msg[0], covariance_msg[1]],
                            [covariance_msg[2], covariance_msg[3]]], dtype=float)
                        self._ekf.update_with_theta(time,
                                                    state,
                                                    covariance,
                                                    msg.pose.theta,
                                                    msg.box_uncertainty,
                                                    msg.flip_certainty < 0.5)

            elif (self._state == SingleRoombaFilterState.HAVE_ORIENTATION
               or self._state == SingleRoombaFilterState.HAVE_ORIENTATION_STOPPED):
                self._ekf.update_with_theta(
                        time,
                        state,
                        covariance,
                        roomba_detection_msg.pose.theta,
                        roomba_detection_msg.box_uncertainty,
                        roomba_detection_msg.flip_certainty < 0.5)
            else:
                assert False, 'Invalid state in single roomba filter'

            # Stopping logic
            MEASUREMENT_QUEUE_LEN = rospy.Duration(1.0)
            while self._measurements[0][0] < self._measurements[-1][0] - MEASUREMENT_QUEUE_LEN:
                self._ekf.set_oldest_buffer_time(self._measurements[0][0])
                self._measurements.popleft()

            if self._state == SingleRoombaFilterState.HAVE_ORIENTATION:
                positions = np.array(
                        [[msg.pose.x, msg.pose.y] for _, msg in self._measurements],
                        dtype=float)
                # TODO calculate actual position uncertainty
                uncertainties = [math.sqrt(max(msg.position_covariance[0],
                                               msg.position_covariance[3]))
                                 for _, msg in self._measurements]
                timestamps = [time for time, _ in self._measurements]
                theta = self._ekf.get_state()[1][2,0]
                stop_time = stop_detector(positions,
                                          uncertainties,
                                          timestamps,
                                          SingleRoombaFilter.SPEED,
                                          theta)
                if stop_time is not None:
                    # We're stopped
                    self._ekf.set_v(0.0, SingleRoombaFilter.ROTATION_RATE, stop_time)
                    self._rotation_stop_time = \
                            stop_time + SingleRoombaFilter.FULL_ROTATION_TIME
                    if self._rotation_stop_time < time:
                        self._ekf.set_v(SingleRoombaFilter.SPEED,
                                        0.0,
                                        self._rotation_stop_time)
                        self._rotation_stop_time = None
                    else:
                        self._state = SingleRoombaFilterState.HAVE_ORIENTATION_STOPPED
                    self._measurements.clear()

    def get_state(self, time):
        with self._lock:
            self._predict(time)
            if self._state == SingleRoombaFilterState.NO_ORIENTATION:
                _, state, cov = self._kf.get_state()
                pos = state[:2,0]
                pos_cov = cov[:2,:2]
                vel = state[2:,0]
                theta = math.atan2(vel[1], vel[0])
                z_rate = 0
            elif self._state == SingleRoombaFilterState.HAVE_ORIENTATION:
                _, state, cov = self._ekf.get_state()
                pos = state[:2,0]
                pos_cov = cov[:2,:2]
                theta = state[2,0]
                vel = SingleRoombaFilter.SPEED * np.array([math.cos(theta),
                                                           math.sin(theta)])
                z_rate = state[3,0]
            elif self._state == SingleRoombaFilterState.HAVE_ORIENTATION_STOPPED:
                _, state, cov = self._ekf.get_state()
                pos = state[:2,0]
                pos_cov = cov[:2,:2]
                theta = state[2,0]
                vel = np.array([0.0, 0.0])
                z_rate = state[3,0]
            else:
                assert False

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

            return odom
