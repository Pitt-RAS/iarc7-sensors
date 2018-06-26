from __future__ import division

import threading

import math
import numpy as np

class KalmanFilterNotInitializedException(Exception):
    pass

class KalmanFilter2d(object):
    '''
    State [x, y, vx, vy]
    '''

    def __init__(self):
        self._lock = threading.RLock()
        with self._lock:
            self._last_time = None

            # Initial uncertainty
            self._initial_P = np.array((
                (0.01, 0, 0, 0),
                (0, 0.01, 0, 0),
                (0, 0, 10, 0),
                (0, 0, 0, 10)), dtype=float)

            # Process noise
            self._Q = np.array((
                (0.05,  0,  0,  0 ),
                (0,  0.05,  0,  0 ),
                (0,  0,  3,  0 ),
                (0,  0,  0,  3 )), dtype=float)

            self._H = np.array((
                (1, 0, 0, 0),
                (0, 1, 0, 0)), dtype=float)

    def orientation(self):
        with self._lock:
            vel = self._s[2:,0]
            vel_covariance = self._P[2:,2:]
            angle_estimate = math.atan2(vel[1], vel[0])

            # Wrap theta between 0 and 2pi
            angle_estimate %= 2*np.pi
            assert angle_estimate >= 0 and angle_estimate <= 2*np.pi

            variances = np.linalg.eigvalsh(vel_covariance)
            stddev = np.sqrt(variances)
            stddev_max = np.max(stddev)

            if 3*stddev_max < np.linalg.norm(vel):
                return angle_estimate, stddev_max / np.linalg.norm(vel)
            else:
                return None, None

    def initialized(self):
        return self._last_time is not None

    def get_state(self):
        '''
        Returns (last_time, state)
        '''
        if not self.initialized():
            raise KalmanFilterNotInitializedException()
        return self._last_time, np.copy(self._s), np.copy(self._P)

    def predict(self, time):
        if not self.initialized():
            raise KalmanFilterNotInitializedException()
        with self._lock:
            dt = (time - self._last_time).to_sec()

            if dt < 0:
                raise Exception(
                        'KalmanFilter2d asked to predict into the past: %.3f %.3f'
                        % (self._last_time.to_sec(), time.to_sec()))

            if dt == 0:
                return

            F = np.array((
                (1,  0,  dt, 0 ),
                (0,  1,  0,  dt),
                (0,  0,  1,  0 ),
                (0,  0,  0,  1 )), dtype=float)
            Q = self._Q * dt

            self._s = F.dot(self._s)
            self._P = F.dot(self._P).dot(F.T) + Q

            self._last_time = time

    def set_state(self, time, state):
        with self._lock:
            self._s = state
            self._P = self._initial_P
            self._last_time = time

    def update(self, time, pos, covariance):
        with self._lock:
            if pos.shape != (2,):
                raise Exception('Position must have shape (2,)')

            if not self.initialized():
                self.set_state(
                        time,
                        np.expand_dims(
                            np.concatenate([pos, np.zeros((2,))]), axis=-1))
                return

            dt = (time - self._last_time).to_sec()

            if dt < 0:
                raise Exception(
                        'KalmanFilter2d received messages less than 0ns apart: %.3f %.3f'
                        % (self._last_time.to_sec(), time.to_sec()))

            self.predict(time)

            R = covariance
            assert R.shape == (2,2)
            H = self._H

            z = np.expand_dims(pos, axis=-1)
            innov = z - H.dot(self._s)
            innov_cov = R + H.dot(self._P).dot(H.T)
            K = self._P.dot(H.T).dot(np.linalg.inv(innov_cov))

            self._s = self._s + K.dot(innov)
            self._P = (np.eye(4) - K.dot(H)).dot(self._P)
