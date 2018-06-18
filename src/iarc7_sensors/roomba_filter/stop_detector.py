from __future__ import division

import math
import numpy as np

def stop_detector(positions, position_uncertainties, timestamps, v, theta):
    '''
    Attempts fits to the data for a moving target (at speed v in direction
    theta) that suddenly stops.  The moving target has equations of motion

    x = x0 + cos(theta)*v*t
    y = y0 + sin(theta)*v*t
    '''
    MIN_OBSERVED_STOPPED_POINTS = 5
    DETECTION_FACTOR = 5

    N = len(positions)

    vx = math.cos(theta)*v
    vy = math.sin(theta)*v
    v = np.array([vx, vy], dtype=float)

    t0 = min(timestamps)
    time_offsets = np.array([(t - t0).to_sec() for t in timestamps], dtype=float)

    v_times_t = np.expand_dims(time_offsets, axis=-1) * v

    position_uncertainties = np.max(1e-16, position_uncertainties)
    weights = 1 / position_uncertainties**2
    expanded_weights = np.expand_dims(weights, axis=-1)
    sum_weights = np.sum(weights)

    best_stddev = float('Inf')
    best_average = None
    best_stop_time = None
    for split_candidate in range(N - MIN_OBSERVED_STOPPED_POINTS):
        if split_candidate == 0:
            t_stop = time_offsets[0]
        else:
            t_stop = (time_offsets[split_candidate-1]
                    + time_offsets[split_candidate]) / 2

        p0_estimates = positions - np.concatenate(
                [v_times_t[:split_candidate],
                 np.broadcast_to(v*t_stop, (N-split_candidate,2))], axis=0)

        p0_average = np.sum(expanded_weights * p0_estimates, axis=0) / sum_weights

        p0_stddev = np.sqrt(np.sum(expanded_weights * (p0_estimates - p0_average)**2, axis=0)
                          / ((N-1)/N*sum_weights))

        if p0_stddev < best_stddev:
            best_stddev = p0_stddev
            best_average = p0_average
            best_stop_time = t_stop

    p0_estimates = positions - v_times_t
    p0_average = np.sum(expanded_weights * p0_estimates, axis=0) / sum_weights
    p0_stddev = np.sqrt(np.sum(expanded_weights * (p0_estimates - p0_average)**2, axis=0)
                      / ((N-1)/N*sum_weights))

    if p0_stddev > DETECTION_FACTOR * best_stddev:
        # Stop detected
        return t_stop
    else:
        return None
