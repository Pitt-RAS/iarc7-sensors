from __future__ import division

import math
import numpy as np
import rospy
import threading

class KalmanFilterNotInitializedException(Exception):
    pass

class ExtendedKalmanFilter2d(object):
    '''
    State [x, y, theta, theta_dot]

    For equations, see doc/roomba_filter/roomba_filter.pdf
    '''

    def __init__(self, v):
        self._lock = threading.RLock()
        with self._lock:
            self._last_time = None

            self._v = v

            # Initial uncertainty
            self._initial_P = np.array((
                (0.5,    0,    0,     0),
                (0,    0.5,    0,     0),
                (0,      0,    1,     0),
                (0,      0,    0, 100.0)), dtype=float)

            # Process noise
            self._Q = np.array((
                (0.5,    0,  0,  0),
                (0,    0.5,  0,  0),
                (0,       0,  1,  0),
                (0,       0,  0,  10)), dtype=float)

            self._H_pos = np.array((
                (1, 0, 0, 0),
                (0, 1, 0, 0)), dtype=float)

            self._H_pos_with_angle = np.array((
                (1, 0, 0, 0),
                (0, 1, 0, 0),
                (0, 0, 1, 0),), dtype=float)

            # Measurement noise
            self._R_pos = np.array((
                (0.01, 0),
                (0, 0.01)), dtype=float)

            self._R_pos_with_angle = np.array((
                (0.01,    0,  0),
                (0,    0.01,  0),
                (0,       0,  0.1),), dtype=float)

    def initialized(self):
        return self._last_time is not None

    def get_state(self):
        '''
        Returns (last_time, state)
        '''
        if not self.initialized():
            raise KalmanFilterNotInitializedException()
        return self._last_time, np.copy(self._s)

    def _jacobian(self, state, tau):
        x0 = state[0,0]
        y0 = state[1,0]
        theta0 = state[2,0]
        theta_dot0 = state[3,0]

        sin = math.sin
        cos = math.cos

        st = sin(theta0)
        ct = cos(theta0)

        stdot = sin(theta_dot0 * tau)
        ctdot = cos(theta_dot0 * tau)

        J = np.eye(4)

        if abs(theta_dot0) > 1e-3:
            # dx / dtheta0
            J[0, 2] = self._v * (-st*stdot/theta_dot0 + ct/theta_dot0*(ctdot - 1))

            # dx / dtheta_dot0
            J[0, 3] = self._v * (ct * (tau*ctdot/theta_dot0 - stdot/theta_dot0**2)
                               + st * (tau*-stdot/theta_dot0 - (ctdot-1)/theta_dot0**2))

            # dy / dtheta0
            J[1, 2] = self._v * (ct*stdot/theta_dot0 + st/theta_dot0*(ctdot-1))

            # dy / dtheta_dot0
            J[1, 3] = self._v * (st * (tau*ctdot/theta_dot0 - stdot/theta_dot0**2)
                               - ct * (tau*-stdot/theta_dot0 - (ctdot-1)/theta_dot0**2))
        else:
            # Expansion to first order in theta_dot

            # dx / dtheta0
            J[0, 2] = self._v * (-st*tau)

            # dx / dtheta_dot0
            J[0, 3] = self._v * (ct * (-1/6 * tau**3 * theta_dot0)
                               + st * (-0.5 * tau**2))

            # dy / dtheta0
            J[1, 2] = self._v * (ct*tau)

            # dy / dtheta_dot0
            J[1, 3] = self._v * (st * (-1/6 * tau**3 * theta_dot0)
                               - ct * (-0.5 * tau**2))

        # dtheta / dtheta_dot0
        J[2, 3] = tau

        return J

    def predict(self, time):
        if not self.initialized():
            raise KalmanFilterNotInitializedException()
        with self._lock:
            assert self._s.shape == (4,1)

            dt = (time - self._last_time).to_sec()

            if dt < 0:
                rospy.logerr(
                        'ExtendedKalmanFilter2d asked to predict into the past: %s %s',
                        self._last_time,
                        time)
                return

            if dt == 0:
                return

            self._s[0,0] += math.cos(self._s[2,0]) * self._v * dt
            self._s[1,0] += math.sin(self._s[2,0]) * self._v * dt
            self._s[2,0] += self._s[3,0] * dt

            # Wrap theta between 0 and 2pi
            self._s[2,0] %= 2*np.pi
            assert self._s[2,0] >= 0 and self._s[2,0] <= 2*np.pi

            J = self._jacobian(self._s, dt)
            self._P = J.dot(self._P).dot(J.T) + self._Q*dt**2

            self._last_time = time

    def set_state(self, time, state):
        assert state.shape == (4,1)
        with self._lock:
            self._s = state
            self._P = self._initial_P
            self._last_time = time

    def update(self, time, pos):
        if not self.initialized():
            raise KalmanFilterNotInitializedException()
        with self._lock:
            assert self._s.shape == (4,1)
            assert pos.shape == (2,)

            dt = (time - self._last_time).to_sec()

            if dt <= 0:
                rospy.logerr(
                        'SimpleRoombaFilter received messages less than 0ns apart: %s %s',
                        self._last_time,
                        time)
                return

            self.predict(time)
            assert self._last_time == time

            R = self._R_pos
            H = self._H_pos

            z = np.expand_dims(pos, axis=-1)
            innov = z - H.dot(self._s)
            innov_cov = R + H.dot(self._P).dot(H.T)
            K = self._P.dot(H.T).dot(np.linalg.inv(innov_cov))

            self._s = self._s + K.dot(innov)
            self._P = (np.eye(4) - K.dot(H)).dot(self._P)

            # Wrap theta between 0 and 2pi
            self._s[2,0] %= 2*np.pi
            assert self._s[2,0] >= 0 and self._s[2,0] <= 2*np.pi

    def update_with_theta(self,
                          time,
                          pos,
                          theta,
                          theta_uncertainty,
                          allow_flip=False):
        if not self.initialized():
            state = np.expand_dims(np.concatenate([pos, np.array([theta, 0])]),
                                   axis=-1)
            self.set_state(time, state)
            return
        with self._lock:
            assert self._s.shape == (4,1)
            assert pos.shape == (2,)

            dt = (time - self._last_time).to_sec()

            if dt <= 0:
                rospy.logerr(
                        'SimpleRoombaFilter received messages less than 0ns apart: %s %s',
                        self._last_time,
                        time)
                return

            self.predict(time)
            assert self._last_time == time

            R = self._R_pos_with_angle.copy()
            R[2,2] = theta_uncertainty**2

            H = self._H_pos_with_angle

            z = np.expand_dims(np.concatenate(
                [pos, np.array([theta], dtype=float)]), axis=-1)
            innov = z - H.dot(self._s)

            # Wrap theta update between -pi and pi
            if innov[2,0] < -np.pi:
                innov[2,0] += 2*np.pi
            if innov[2,0] > np.pi:
                innov[2,0] -= 2*np.pi
            assert innov[2,0] >= -np.pi and innov[2,0] <= np.pi

            if allow_flip:
                # Wrap theta update between -pi/2 and pi/2
                if innov[2,0] < -np.pi/2:
                    innov[2,0] += np.pi
                if innov[2,0] > np.pi/2:
                    innov[2,0] -= np.pi
                assert innov[2,0] >= -np.pi/2 and innov[2,0] <= np.pi/2

            innov_cov = R + H.dot(self._P).dot(H.T)
            K = self._P.dot(H.T).dot(np.linalg.inv(innov_cov))

            self._s = self._s + K.dot(innov)
            self._P = (np.eye(4) - K.dot(H)).dot(self._P)

            # Wrap theta between 0 and 2pi
            self._s[2,0] %= 2*np.pi
            assert self._s[2,0] >= 0 and self._s[2,0] <= 2*np.pi
