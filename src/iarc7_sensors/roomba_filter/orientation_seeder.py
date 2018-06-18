from __future__ import division

import threading

import math
import numpy as np
import rospy

class OrientationSeeder(object):
    def __init__(self):
        self._lock = threading.Lock()
        with self._lock:
            self._converged = False
            self._estimates = []
            self._uncertainties = []

    def add_orientation_estimate(self, estimate, uncertainty):
        assert uncertainty >= 0
        if uncertainty == 0:
            uncertainty = 1e-16

        with self._lock:
            # The estimates are of angles, what we do here is convert them
            # to unit vectors with uncertainties in 2d space
            self._estimates.append(np.array([math.cos(estimate),
                                             math.sin(estimate)],
                                            dtype=float))
            self._uncertainties.append(uncertainty)

            self._update_orientation_estimate()

    def converged(self):
        return self._converged

    def get_estimate(self):
        with self._lock:
            if not self._converged:
                raise Exception('Attempt to get orientation seed before convergence')
            return self._angle_estimate, self._uncertainty

    def _update_orientation_estimate(self):
        estimates = np.stack(self._estimates)
        expanded_weights = np.expand_dims(1 / np.array(self._uncertainties)**2, axis=-1)
        weighted_estimates = expanded_weights * estimates
        sum_weights = np.sum(expanded_weights)
        average = np.sum(weighted_estimates, axis=0) / sum_weights

        N = len(self._estimates)

        # TODO: replace with covariance matrix
        if N < 2:
            stddev = np.array([float('Inf'), float('Inf')])
        else:
            stddev = np.sqrt(
                    np.sum(expanded_weights * (estimates - average)**2, axis=0)
                  / ((N-1)/N*sum_weights))
        stddev_max = np.max(stddev)

        self._angle_estimate = math.atan2(average[1], average[0])

        # Wrap theta between 0 and 2pi
        self._angle_estimate %= 2*np.pi
        assert self._angle_estimate >= 0 and self._angle_estimate <= 2*np.pi

        self._converged = 3*stddev_max < np.linalg.norm(average)

        # TODO: calculate based on variance perpendicular to origin
        self._uncertainty = stddev_max / np.linalg.norm(average)
