from __future__ import division

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

class SingleRoombaFilterState(Enum):
    NO_ORIENTATION = 0
    HAVE_ORIENTATION = 1

class SingleRoombaFilter(object):
    def __init__(self, world_fixed_frame):
        self._lock = threading.Lock()
        with self._lock:
            self._id = uuid.uuid4()
            self._world_fixed_frame = world_fixed_frame
            self._frame = 'roomba-{}/base_link'.format(self._id)
            self._world_fixed_frame = world_fixed_frame
            self._state = SingleRoombaFilterState.NO_ORIENTATION
            self._v = 0.3
            self._kf = KalmanFilter2d()
            self._ekf = ExtendedKalmanFilter2d(self._v)
            self._orientation_seeder = OrientationSeeder()

    def add_measurement(self, time, roomba_detection_msg):
        with self._lock:
            state = np.array([roomba_detection_msg.pose.x,
                              roomba_detection_msg.pose.y], dtype=float)

            if self._state == SingleRoombaFilterState.NO_ORIENTATION:
                self._kf.update(time, state)
                if roomba_detection_msg.flip_certainty > 0.5:
                    self._orientation_seeder.add_orientation_estimate(
                            roomba_detection_msg.pose.theta,
                            roomba_detection_msg.box_uncertainty)

                orientation = None
                orientation_uncertainty = float('Inf')

                kf_orientation_estimate, kf_uncertainty = self._kf.orientation()
                if kf_orientation_estimate is not None:
                    rospy.logwarn('KF ORIENTATION CONVERGED')
                    orientation = kf_orientation_estimate
                    orientation_uncertainty = kf_uncertainty

                if self._orientation_seeder.converged():
                    rospy.logwarn('ORIENTATION SEEDER CONVERGED')
                    seeder_orientation_estimate, seeder_uncertainty = \
                            self._orientation_seeder.get_estimate()
                    if seeder_uncertainty < orientation_uncertainty:
                        orientation = seeder_orientation_estimate
                        orientation_uncertainty = seeder_uncertainty

                if orientation is not None:
                    self._state = SingleRoombaFilterState.HAVE_ORIENTATION
                    _, kf_state = self._kf.get_state()
                    self._ekf.update_with_theta(
                            time,
                            kf_state[:2,0],
                            orientation,
                            roomba_detection_msg.flip_certainty < 0.5)

            elif self._state == SingleRoombaFilterState.HAVE_ORIENTATION:
                self._ekf.update_with_theta(time,
                                            state,
                                            roomba_detection_msg.pose.theta,
                                            roomba_detection_msg.box_uncertainty)
            else:
                assert False, 'Invalid state in single roomba filter'

            rospy.loginfo(self._state.name)

    def get_state(self, time):
        with self._lock:
            if self._state == SingleRoombaFilterState.NO_ORIENTATION:
                self._kf.predict(time)
                _, state = self._kf.get_state()
                pos = state[:2,0]
                vel = state[2:,0]
                theta = math.atan2(vel[1], vel[0])
                z_rate = 0
            else:
                self._ekf.predict(time)
                _, state = self._ekf.get_state()
                pos = state[:2,0]
                theta = state[2,0]
                vel = self._v * np.array([math.cos(theta), math.sin(theta)])
                z_rate = state[3,0]

            odom = Odometry()
            odom.header.stamp = time
            odom.header.frame_id = self._world_fixed_frame
            odom.child_frame_id = self._frame
            odom.pose.pose.position.x = pos[0]
            odom.pose.pose.position.y = pos[1]

            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            odom.pose.pose.orientation.x = quat[0]
            odom.pose.pose.orientation.y = quat[1]
            odom.pose.pose.orientation.z = quat[2]
            odom.pose.pose.orientation.w = quat[3]

            odom.twist.twist.linear.x = vel[0]
            odom.twist.twist.linear.y = vel[1]
            odom.twist.twist.angular.z = z_rate

            return odom
