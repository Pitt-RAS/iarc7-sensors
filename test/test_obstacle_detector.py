import unittest

import numpy as np

import imp
import os
import os.path
obstacle_detector = imp.load_source(
        'obstacle_detector',
        os.path.abspath(os.path.join(os.path.dirname(__file__),
                                     os.pardir,
                                     'src',
                                     'iarc7_sensors',
                                     'obstacle_detector.py')))

def test_get_rotation_matrices():
    from geometry_msgs.msg import Quaternion
    import math
    import numpy as np
    from tf import transformations

    q0 = np.array((0.0, 0.0, 0.0, 1.0))
    q1 = transformations.quaternion_about_axis(math.pi, (0, 0, 1))

    result = obstacle_detector.get_rotation_matrices(q0, q1, np.array((0.0, 0.5, 1.0)))

    assert result.shape == (3, 4, 4)

    result_vector = np.einsum('tij,j->ti', result, np.array((1, 0, 0, 1)))

    print result_vector
    assert np.allclose(result_vector[0], np.array((1, 0, 0, 1)))
    assert np.allclose(result_vector[1], np.array((0, 1, 0, 1)))
    assert np.allclose(result_vector[2], np.array((-1, 0, 0, 1)))

    result_vector = np.einsum('tij,j->ti', result, np.array((1, 2, 3, 1)))

    assert np.allclose(result_vector[0], np.array((1, 2, 3, 1)))
    assert np.allclose(result_vector[1], np.array((-2, 1, 3, 1)))
    assert np.allclose(result_vector[2], np.array((-1, -2, 3, 1)))

def test_get_translation_matrices():
    v1 = np.array([1.0, 2.0, 3.0])
    v2 = np.array([4.0, 5.0, 6.0])

    result = obstacle_detector.get_translation_matrices(v1, v2, np.array((0.0, 0.5, 1.0)))

    assert result.shape == (3, 4, 4)

    assert np.allclose(result, np.array((
        (
            (1.0, 0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0, 2.0),
            (0.0, 0.0, 1.0, 3.0),
            (0.0, 0.0, 0.0, 1.0)
            ),
        (
            (1.0, 0.0, 0.0, 2.5),
            (0.0, 1.0, 0.0, 3.5),
            (0.0, 0.0, 1.0, 4.5),
            (0.0, 0.0, 0.0, 1.0)
            ),
        (
            (1.0, 0.0, 0.0, 4.0),
            (0.0, 1.0, 0.0, 5.0),
            (0.0, 0.0, 1.0, 6.0),
            (0.0, 0.0, 0.0, 1.0)
            )
        )))
