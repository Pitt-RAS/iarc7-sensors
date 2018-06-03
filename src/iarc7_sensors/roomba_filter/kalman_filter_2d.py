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
            self._P = np.array((
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

            # Measurement noise
            self._R = np.array((
                (0.05, 0),
                (0, 0.05)), dtype=float)

    def initialized(self):
        return self._last_time is not None

    def get_state(self):
        if not self.initialized():
            raise KalmanFilterNotInitializedException()
        return np.copy(self._s)

    def predict(self, time):
        if not self.initialized():
            raise KalmanFilterNotInitializedException()
        with self._lock:
            dt = (time - self._last_time).to_sec()

            if dt < 0:
                rospy.logerr(
                        'KalmanFilter2d asked to predict into the past: %s %s',
                        self._last_time,
                        time)
                return

            if dt == 0:
                return

            F = np.array((
                (1,  0,  dt, 0 ),
                (0,  1,  0,  dt),
                (0,  0,  1,  0 ),
                (0,  0,  0,  1 )), dtype=float)
            Q = self._Q * dt**2

            self._s = F.dot(self._s)
            self._P = F.dot(self._P).dot(F.T) + Q

            self._last_time = time

    def set_state(self, time, state):
        with self._lock:
            self._s = state
            self._last_time = time

    def update(self, time, pos):
        if not self.initialized():
            raise KalmanFilterNotInitializedException()
        with self._lock:
            dt = (msg.header.stamp - self._last_time).to_sec()

            if dt <= 0:
                rospy.logerr(
                        'SimpleRoombaFilter received messages less than 0ns apart: %s %s',
                        self._last_time,
                        msg.header.stamp)
                return

            self.predict(time)

            z = np.expand_dims(pos, axis=-1)
            innov = z - H.dot(self._s)
            innov_cov = R + H.dot(self._P).dot(H.T)
            K = self._P.dot(H.T).dot(np.linalg.inv(innov_cov))

            self._s = self._s + K.dot(innov)
            self._P = (np.eye(4) - K.dot(H)).dot(self._P)
