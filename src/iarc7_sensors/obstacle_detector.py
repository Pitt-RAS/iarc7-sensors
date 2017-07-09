#!/usr/bin/env python2
import rospy

from sensor_msgs.msg import LaserScan

import tf2_ros

from collections import deque
import numpy as np
import numpy.linalg as linalg
import scipy.interpolate
import scipy.optimize

def slerp(q0, q1, t):
    q0 = q0 / linalg.norm(q0)
    q1 = q1 / linalg.norm(q1)

    dot = q0.dot(q1)

    if abs(dot) > 0.9995:
        result = q0 + np.outer(t, q1 - q0)
        return result / linalg.norm(result)

    if dot < 0:
        q1 *= -1
        dot *= -1

    dot = min(1, max(dot, -1))
    theta = t * np.arccos(dot)
    q2 = q1 - q0*dot
    q2 /= linalg.norm(q2)
    return np.outer(np.cos(theta), q0) + np.outer(np.sin(theta), q2)

def scan_callback(msg):
    if msg_queue and msg_queue[-1].header.stamp <= msg.header.stamp:
        msg_queue.append(msg)
    else:
        rospy.logerr('Rejecting scan with old timestamp: %s'%msg.header.stamp)

def get_circles(points, radius):
    def f(x):
        return linalg.norm(points - x, axis=1) - radius

    def Dfun(x):
        return (x-points) / linalg.norm(points - x, axis=1)

    x, cov_x, _ = scipy.optimize.leastsq(f,
                                         x0=np.mean(points, axis=0),
                                         Dfun=Dfun,
                                         col_deriv=True)
    # TODO: use cov_x to decide if fitting was successful and if we should try
    # fitting multiple circles
    return (x,)

def get_rotation_matrices(orientation_start, orientation_end, samples):
    orientations = slerp(orientation_start,
                         orientation_end,
                         np.linspace(0.0, 1.0, samples))

    q = np.einsum('ti,tj->tij', orientations, orientations)
    rotation_matrices = np.array((
        (1.0-q[:,1,1]-q[:,2,2],     q[:,0,1]-q[:,2,3],     q[:,0,2]+q[:,1,3], 0.0),
        (    q[:,0,1]+q[:,2,3], 1.0-q[:,0,0]-q[:,2,2],     q[:,1,2]-q[:,0,3], 0.0),
        (    q[:,0,2]-q[:,1,3],     q[:,1,2]+q[:,0,3], 1.0-q[:,0,0]-q[:,1,1], 0.0),
        (                  0.0,                   0.0,                   0.0, 1.0)))
    rotation_matrices = np.einsum('ijt->tij', rotation_matrices)

    return rotation_matrices

def get_transform_matrices(tf_start, tf_end, samples):
    translation_start = np.array([tf_start.transform.translation.x,
                                  tf_start.transform.translation.y,
                                  tf_start.transform.translation.z])
    translation_end = np.array([tf_end.transform.translation.x,
                                  tf_end.transform.translation.y,
                                  tf_end.transform.translation.z])

    orientation_start = np.array([tf_start.transform.rotation.w,
                                  tf_start.transform.rotation.x,
                                  tf_start.transform.rotation.y,
                                  tf_start.transform.rotation.z])
    orientation_end = np.array([tf_end.transform.rotation.w,
                                tf_end.transform.rotation.x,
                                tf_end.transform.rotation.y,
                                tf_end.transform.rotation.z])

    translation_matrices = get_translation_matrices(translation_start,
                                                    translation_end,
                                                    samples)
    rotation_matrices = get_rotation_matrices(orientation_start,
                                              orientation_end,
                                              samples)

    transform_matrices = np.dot(translation_matrices, rotation_matrices)
    return transform_matrices

def get_translation_matrices(translation_start, translation_end, samples):
    translations = (scipy.interpolate.interp1d(
            np.array([0.0, 1.0]),
            np.vstack((translation_start, translation_end)),
            axis=0))(np.linspace(0.0, 1.0, samples))

    translation_matrices = np.tile(np.eye(4), np.array([samples, 1, 1]))
    translation_matrices[:,0:3,3] = translations

    return translation_matrices

def process_scan(scan, tf_start, tf_end, settings):
    ranges = np.array(scan.ranges)
    angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

    range_mask = (ranges >= scan.range_min
                & ranges <= scan.range_max
                & ranges >= drone_radius)

    range_points = ranges * np.array([np.cos(angles), np.sin(angles), 0.0])
    range_points = np.einsum('it->ti', range_points)

    transform_matrices = get_transform_matrices(tf_start, tf_end, len(ranges))

    transformed_range_points = np.einsum('tij,tj->ti',
                                         transform_matrices,
                                         range_points)

    mask = (transformed_range_points[:,0] >= settings.min_x
          & transformed_range_points[:,0] <= settings.max_x
          & transformed_range_points[:,1] >= settings.min_y
          & transformed_range_points[:,1] <= settings.max_y
          & transformed_range_points[:,2] >= settings.min_z
          & transformed_range_points[:,2] <= settings.max_z)

if __name__ == '__main__':
    rospy.init_node('obstacle_detector')
    rospy.Subscriber('scan', LaserScan, scan_callback)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    msg_queue = deque()

    while not rospy.is_shutdown():
        # TODO: make this efficient
        if msg_queue:
            msg = msg_queue.popleft()
            tf_start = tf_buffer.lookup_transform('quad',
                                                  msg.header.frame_id,
                                                  msg.header.stamp,
                                                  timeout)
            tf_end = tf_buffer.lookup_transform(
                    'quad',
                    msg.header.frame_id,
                    msg.header.stamp + rospy.Duration(msg.time_increment * (len(msg.ranges) - 1)),
                    timeout)
            process_scan(msg, tf_start, tf_end)
