from __future__ import division

from collections import deque
import math
import numpy as np
import threading

class KalmanFilterNotInitializedException(Exception):
    pass

class StateEvent(object):
    def __init__(self, time, state, covariance):
        self.time = time
        self.state = state
        self.covariance = covariance

class PositionMeasurementEvent(object):
    def __init__(self, time, *measurement_args):
        self.time = time
        self.measurement_args = measurement_args

class PositionAndThetaMeasurementEvent(object):
    def __init__(self, time, *measurement_args):
        self.time = time
        self.measurement_args = measurement_args

class ThetaDotMeasurementEvent(object):
    def __init__(self, time, *measurement_args):
        self.time = time
        self.measurement_args = measurement_args

class ExtendedKalmanFilter2d(object):
    '''
    State [x, y, theta, theta_dot]

    For equations, see doc/roomba_filter/roomba_filter.pdf
    '''

    def __init__(self, v):
        self._lock = threading.Lock()
        with self._lock:
            # Includes both `StateEvent`s and `MeasurementEvent`s
            self._event_buffer = deque()
            self._last_time = None

            self._v = v

            # Initial uncertainty
            self._initial_P = np.array((
                (  0.5,    0,    0,    0),
                (    0,  0.5,    0,    0),
                (    0,    0,  0.5,    0),
                (    0,    0,    0, 1e-1)), dtype=float)

            # Process noise
            self._Q = np.array((
                ( 1e-4,    0,    0,    0),
                (    0, 1e-4,    0,    0),
                (    0,    0, 1e-4,    0),
                (    0,    0,    0, 1e-3)), dtype=float)

            self._H_pos = np.array((
                (1, 0, 0, 0),
                (0, 1, 0, 0)), dtype=float)

            self._H_pos_with_angle = np.array((
                (1, 0, 0, 0),
                (0, 1, 0, 0),
                (0, 0, 1, 0),), dtype=float)

            self._H_angle_dot = np.array((
                (0, 0, 0, 1),), dtype=float)

            # Measurement noise
            self._R_pos_with_angle = np.array((
                (float('NaN'), float('NaN'),  0           ),
                (float('NaN'), float('NaN'),  0           ),
                (0,            0,             float('NaN')),), dtype=float)

            self._R_angle_dot = np.array((
                (float('NaN'),),), dtype=float)

    def _append_buffer(self, event):
        MAX_QUEUE_LENGTH = 500

        if self._event_buffer and event.time < self._event_buffer[-1].time:
            raise Exception('ExtendedKalmanFilter2d event buffer not monotonic')
        self._event_buffer.append(event)
        while len(self._event_buffer) > MAX_QUEUE_LENGTH:
            self._event_buffer.popleft()

    def _initialized(self):
        return self._last_time is not None

    def get_state(self):
        '''
        Returns (last_time, state)
        '''
        with self._lock:
            if not self._initialized():
                raise KalmanFilterNotInitializedException()
            return self._last_time, np.copy(self._s), np.copy(self._P)

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
        with self._lock:
            if not self._initialized():
                raise KalmanFilterNotInitializedException()
            self._predict(time)
            self._append_buffer(StateEvent(time, self._s.copy(), self._P.copy()))

    def _predict(self, time):
        assert self._s.shape == (4,1)

        dt = (time - self._last_time).to_sec()

        if dt < 0:
            raise Exception(
                    ('ExtendedKalmanFilter2d asked to predict to time %.3f,'
                   + ' when last time was %.3f')
                    % (time.to_sec(), self._last_time.to_sec()))

        if dt == 0:
            return

        self._s[0,0] += math.cos(self._s[2,0]) * self._v * dt
        self._s[1,0] += math.sin(self._s[2,0]) * self._v * dt
        self._s[2,0] += self._s[3,0] * dt

        # Wrap theta between 0 and 2pi
        self._s[2,0] %= 2*np.pi
        assert self._s[2,0] >= 0 and self._s[2,0] <= 2*np.pi

        J = self._jacobian(self._s, dt)
        self._P = J.dot(self._P).dot(J.T) + self._Q*dt

        self._last_time = time

    def set_v(self, v, omega, time):
        '''
        Set linear and angular velocity at given time
        '''
        with self._lock:
            for i, event in zip(xrange(len(self._event_buffer) - 1, -1, -1),
                                reversed(self._event_buffer)):
                if type(event) == StateEvent and event.time < time:
                    reset_state = event
                    break
            else:
                raise Exception(('Attempted to set velocity in EKF for time %.3f'
                              + ' earlier than beginning of state buffer at %.3f')
                                % (time.to_sec(), self._event_buffer[0].time.to_sec()))

            self._last_time = reset_state.time
            self._s = reset_state.state.copy()
            self._P = reset_state.covariance.copy()

            i += 1

            # TODO: This logic is ok given the current usage of this class in
            # SingleRoombaFilter, but won't fly for edge cases not seen there
            while i < len(self._event_buffer) and self._event_buffer[i].time <= time:
                event = self._event_buffer[i]
                if type(event) == StateEvent:
                    event.time = self._last_time
                    event.state = self._s.copy()
                    event.covariance = self._P.copy()
                elif type(event) == PositionMeasurementEvent:
                    self._predict(event.time)
                    self._update(event.time, *event.measurement_args)
                elif type(event) == PositionAndThetaMeasurementEvent:
                    self._predict(event.time)
                    self._update_with_theta(event.time, *event.measurement_args)
                elif type(event) == ThetaDotMeasurementEvent:
                    self._predict(event.time)
                    self._update_theta_dot(event.time, *event.measurement_args)
                else:
                    assert False
                i += 1

            self._predict(time)
            assert self._last_time == time
            self._v = v
            self._s[3,0] = omega

            for i in range(i, len(self._event_buffer)):
                event = self._event_buffer[i]
                if type(event) == StateEvent:
                    event.time = self._last_time
                    event.state = self._s.copy()
                    event.covariance = self._P.copy()
                elif type(event) == PositionMeasurementEvent:
                    self._predict(event.time)
                    self._update(event.time, *event.measurement_args)
                elif type(event) == PositionAndThetaMeasurementEvent:
                    self._predict(event.time)
                    self._update_with_theta(event.time, *event.measurement_args)
                elif type(event) == ThetaDotMeasurementEvent:
                    self._predict(event.time)
                    self._update_theta_dot(event.time, *event.measurement_args)
                else:
                    assert False

    def update(self, time, pos, covariance):
        with self._lock:
            if not self._initialized():
                raise KalmanFilterNotInitializedException()
            self._append_buffer(StateEvent(self._last_time, self._s.copy(), self._P.copy()))
            self._update(time, pos, covariance)
            self._append_buffer(PositionMeasurementEvent(time, pos, covariance))

    def _update(self, time, pos, covariance):
        assert self._s.shape == (4,1)
        assert pos.shape == (2,)
        assert covariance.shape == (2,2)

        dt = (time - self._last_time).to_sec()

        if dt < 0:
            raise Exception(
                    'ExtendedKalmanFilter2d received messages less than 0ns apart: %.3f %.3f'
                    % (self._last_time.to_sec(), time.to_sec()))

        self._predict(time)
        assert self._last_time == time

        R = covariance
        H = self._H_pos.copy()

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
                          pos_covariance,
                          theta,
                          theta_uncertainty,
                          allow_flip):
        if theta_uncertainty > 0.3 and self._initialized():
            self.update(time, pos, pos_covariance)
            return
        with self._lock:
            if not self._initialized():
                state = np.expand_dims(
                             np.concatenate([pos, np.array([theta, 0])]),
                             axis=-1)
                self._s = state.copy()
                self._P = self._initial_P.copy()
                self._P[:3,:3] = self._R_pos_with_angle.copy()
                self._P[:2,:2] = pos_covariance
                self._P[2,2] = theta_uncertainty**2
                self._last_time = time
                self._append_buffer(
                        StateEvent(self._last_time, self._s.copy(), self._P.copy()))
                return
            self._append_buffer(StateEvent(self._last_time, self._s.copy(), self._P.copy()))
            self._update_with_theta(time,
                                    pos,
                                    pos_covariance,
                                    theta,
                                    theta_uncertainty,
                                    allow_flip)
            self._append_buffer(PositionAndThetaMeasurementEvent(
                    time,
                    pos,
                    pos_covariance,
                    theta,
                    theta_uncertainty,
                    allow_flip))

    def _update_with_theta(self,
                           time,
                           pos,
                           pos_covariance,
                           theta,
                           theta_uncertainty,
                           allow_flip):
        assert self._s.shape == (4,1)
        assert pos.shape == (2,)
        assert pos_covariance.shape == (2,2)

        dt = (time - self._last_time).to_sec()

        if dt < 0:
            raise Exception(
                    'ExtendedKalmanFilter2d received messages less than 0ns apart: %.3f %.3f'
                    % (self._last_time.to_sec(), time.to_sec()))

        self._predict(time)
        assert self._last_time == time

        R = self._R_pos_with_angle.copy()
        R[:2,:2] = pos_covariance
        R[2,2] = theta_uncertainty**2 * 4

        H = self._H_pos_with_angle.copy()

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

    def update_theta_dot(self,
                         time,
                         theta_dot,
                         theta_dot_uncertainty):
        with self._lock:
            if not self._initialized():
                raise KalmanFilterNotInitializedException()
            self._update_theta_dot(time,
                                   theta_dot,
                                   theta_dot_uncertainty)

    def _update_theta_dot(self,
                          time,
                          theta_dot,
                          theta_dot_uncertainty):
        assert self._s.shape == (4,1)

        dt = (time - self._last_time).to_sec()

        if dt < 0:
            raise Exception(
                    'ExtendedKalmanFilter2d received messages less than 0ns apart: %.3f %.3f'
                    % (self._last_time.to_sec(), time.to_sec()))

        self._predict(time)
        assert self._last_time == time

        R = self._R_angle_dot.copy()
        R[0,0] = theta_dot_uncertainty**2

        H = self._H_angle_dot.copy()

        z = np.array(((theta_dot,),), dtype=float)
        innov = z - H.dot(self._s)

        innov_cov = R + H.dot(self._P).dot(H.T)
        K = self._P.dot(H.T).dot(np.linalg.inv(innov_cov))

        self._s = self._s + K.dot(innov)
        self._P = (np.eye(4) - K.dot(H)).dot(self._P)

        # Wrap theta between 0 and 2pi
        self._s[2,0] %= 2*np.pi
        assert self._s[2,0] >= 0 and self._s[2,0] <= 2*np.pi
