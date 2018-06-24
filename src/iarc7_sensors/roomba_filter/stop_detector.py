from __future__ import division

import math
import numpy as np
import rospy

def stop_detector(positions, position_uncertainties, timestamps, v, theta):
    '''
    Attempts fits to the data for a moving target (at speed v in direction
    theta) that suddenly stops.  The moving target has equations of motion

    x = x0 + cos(theta)*v*t
    y = y0 + sin(theta)*v*t

    :param positions: numpy array of shape (N,2)
    :param position_uncertainties: numpy array of shape (N,)
    :param timestamps: iterable of rospy.Time objects
    :param v: scalar, expected speed in m/s
    :param theta: scalar expected heading in radians
    '''
    MIN_OBSERVED_STOPPED_POINTS = 10
    DETECTION_FACTOR = 4

    rospy.logdebug('RUNNING STOP DETECTOR')

    N = positions.shape[0]

    if N < MIN_OBSERVED_STOPPED_POINTS:
        return None

    vx = math.cos(theta)*v
    vy = math.sin(theta)*v
    v = np.array([vx, vy], dtype=float)

    t0 = min(timestamps)
    time_offsets = np.array([(t - t0).to_sec() for t in timestamps], dtype=float)

    v_times_t = np.expand_dims(time_offsets, axis=-1) * v

    position_uncertainties = np.maximum(1e-16, position_uncertainties)
    weights = 1 / position_uncertainties**2
    expanded_weights = np.expand_dims(weights, axis=-1)
    sum_weights = np.sum(weights)

    rospy.logdebug('TIMES %.3f to %.3f'%(time_offsets[0], time_offsets[-1]))

    best_stddev = float('Inf')
    best_average = None
    best_stop_time = None
    for split_candidate in range(N - MIN_OBSERVED_STOPPED_POINTS + 1):
        if split_candidate == 0:
            t_stop = time_offsets[0]
        else:
            t_stop = (time_offsets[split_candidate-1]
                    + time_offsets[split_candidate]) / 2

        p0_estimates = positions - np.concatenate(
                [v_times_t[:split_candidate],
                 np.broadcast_to(v*t_stop, (N-split_candidate,2))], axis=0)

        p0_average = np.sum(expanded_weights * p0_estimates, axis=0) / sum_weights

        p0_variances = (np.sum(expanded_weights * (p0_estimates - p0_average)**2, axis=0)
                          / ((N-1)/N*sum_weights))
        p0_sigma_xy = (np.sum(expanded_weights[:,0] * (p0_estimates - p0_average)[:,0]*(p0_estimates - p0_average)[:,1], axis=0)
                          / ((N-1)/N*sum_weights))
        p0_covariance = np.array(((p0_variances[0], p0_sigma_xy), (p0_sigma_xy, p0_variances[1])))
        p0_eigvals = np.linalg.eigvalsh(p0_covariance)
        p0_max_stddev = math.sqrt(np.max(p0_eigvals))

        if p0_max_stddev < best_stddev:
            best_stddev = p0_max_stddev
            best_average = p0_average
            best_stop_time = t_stop

    p0_estimates = positions - v_times_t
    p0_average = np.sum(expanded_weights * p0_estimates, axis=0) / sum_weights

    p0_variances = (np.sum(expanded_weights * (p0_estimates - p0_average)**2, axis=0)
                      / ((N-1)/N*sum_weights))
    p0_sigma_xy = (np.sum(expanded_weights[:,0] * (p0_estimates - p0_average)[:,0]*(p0_estimates - p0_average)[:,1], axis=0)
                      / ((N-1)/N*sum_weights))
    p0_covariance = np.array(((p0_variances[0], p0_sigma_xy), (p0_sigma_xy, p0_variances[1])))
    p0_eigvals = np.linalg.eigvalsh(p0_covariance)
    p0_max_stddev = math.sqrt(np.max(p0_eigvals))

    rospy.logdebug('STDDEV for stop at %.3f: %f'%(best_stop_time, best_stddev))
    rospy.logdebug('STDDEV for no stop: %f'%p0_max_stddev)

    if p0_max_stddev > DETECTION_FACTOR * best_stddev:
        # Stop detected
        rospy.logdebug('STOP AT %.3f'%best_stop_time)
        return t0 + rospy.Duration(best_stop_time)
    else:
        return None
