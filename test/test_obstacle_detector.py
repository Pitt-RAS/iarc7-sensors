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
    v1 = np.array([])

def test_get_translation_matrices():
    v1 = np.array([1.0, 2.0, 3.0])
    v2 = np.array([4.0, 5.0, 6.0])

    result = obstacle_detector.get_translation_matrices(v1, v2, 3)

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
