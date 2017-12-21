#!/usr/bin/env python2

from iarc7_msgs.msg import (Obstacle,
                            ObstacleArray)
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
np.set_printoptions(threshold='nan')

from collections import deque
import numpy.linalg as linalg
import rospy
import scipy.interpolate
import scipy.optimize
from tf import transformations
import tf2_ros
import uuid

'''
Obstacle detector

Detects 2d cylinders using a planar scanning LIDAR, rejecting points outside
of a rectangular volume representing the bounds of the arena and inside of a
cylinder representing the drone

Expects incoming LaserScan messages on the `scan` topic, and publishes
`ObstacleArray` messages on the `obstacles` topic and `MarkerArray` messages
on the `obstacle_markers` topic
'''

def slerp(q0, q1, t):
    '''
    Slerp between quaternions q0 and q1 with samples at times t
    Based on https://en.wikipedia.org/wiki/Slerp#Source_code

    Args:
        q0: start quaternion, sequence of length 4, of the form [x, y, z, w]
        q1: end quaternion, sequence of length 4, of the form [x, y, z, w]
        t: sequence of numbers between 0 and 1 representing times

    Returns:
        orientations: numpy array of shape (len(t), 4), representing the quaternion
                      at each time in t
    '''
    q0 = q0 / linalg.norm(q0)
    q1 = q1 / linalg.norm(q1)

    dot = q0.dot(q1)

    if abs(dot) > 0.9995:
        result = q0 + np.outer(t, q1 - q0)
        return result / np.expand_dims(linalg.norm(result, axis=1), axis=-1)

    if dot < 0:
        q1 *= -1
        dot *= -1

    dot = min(1, max(dot, -1))
    theta = t * np.arccos(dot)
    q2 = q1 - q0*dot
    q2 /= linalg.norm(q2)
    return np.outer(np.cos(theta), q0) + np.outer(np.sin(theta), q2)

def scan_callback(msg):
    if not msg_queue or msg_queue[-1].header.stamp < msg.header.stamp:
        msg_queue.append(msg)
    else:
        rospy.logerr('Rejecting scan with old timestamp: %s'%msg.header.stamp)

def get_best_circle_for_cluster(points, radius, drone_pos):
    '''
    Computes a best-fit circle for a given set of points in 2d

    Args:
        points: numpy array of shape (N, 2), locations of observed points
        radius: radius of circle to fit
        drone_pos: numpy array of shape (2), location of drone

    Returns:
        center: sequence of length 2, center of best-fit circle
    '''
    assert points.size

    # Calculate initial guess for the center, assuming the average of the
    # points is the intersection of the line between the drone and obstacle
    # centers and the bounding circle of the obstacle
    mean_point = np.mean(points, axis=0)
    boundary_point_relative = mean_point - drone_pos
    center_point_relative = (boundary_point_relative
                           * (linalg.norm(boundary_point_relative) + radius)
                           / linalg.norm(boundary_point_relative))
    guess_pos = drone_pos + center_point_relative

    if points.shape[0] == 1:
        # We only have one point, so the fitter won't do any better
        return guess_pos
    else:
        def f(x):
            '''
            Cost function, see documentation for scipy.optimize.leastsq
            '''
            return linalg.norm(x - points, axis=1) - radius

        def Dfun(x):
            '''
            Jacobian of f
            '''
            return ((x-points)
                  / np.expand_dims(linalg.norm(x - points, axis=1),
                                   axis=-1)).T

        x, cov_x = scipy.optimize.leastsq(f,
                                          x0=list(guess_pos),
                                          Dfun=Dfun,
                                          col_deriv=True)
        # TODO: use cov_x to decide if fitting was successful and if we should
        # try fitting multiple circles
        return x

def get_rotation_matrices(orientation_start, orientation_end, samples):
    '''
    Computes a sequence of rotation matrices interpolating between initial
    and final orientations

    This is a vectorized version of the quaternion_matrix function
    in transformations.py, which can be found online here:
    https://www.lfd.uci.edu/~gohlke/code/transformations.py.html

    Args:
        orientation_start: start quaternion, sequence of length 4, of the
                           form [x, y, z, w]
        orientation_end: end quaternion, sequence of length 4, of the
                         form [x, y, z, w]
        samples: sequence of N numbers between 0 and 1 representing times

    Returns:
        numpy array of shape (N, 4, 4), representing a rotation (as an affine
        transformation matrix) for each requested sample
    '''
    assert orientation_start.shape == (4,)
    assert orientation_end.shape == (4,)

    # Interpolated orientations as quaternions
    orientations = slerp(orientation_start,
                         orientation_end,
                         samples)

    assert orientations.shape == (len(samples), 4)

    q = np.einsum('ti,tj->tij', orientations, orientations)
    q *= (2 / linalg.norm(orientations, axis=1)**2)[:, np.newaxis, np.newaxis]

    assert q.shape == (len(samples), 4, 4)

    rotation_matrices = np.array((
        (1.0-q[:,1,1]-q[:,2,2],     q[:,0,1]-q[:,2,3],     q[:,0,2]+q[:,1,3], np.zeros(q.shape[0])),
        (    q[:,0,1]+q[:,2,3], 1.0-q[:,0,0]-q[:,2,2],     q[:,1,2]-q[:,0,3], np.zeros(q.shape[0])),
        (    q[:,0,2]-q[:,1,3],     q[:,1,2]+q[:,0,3], 1.0-q[:,0,0]-q[:,1,1], np.zeros(q.shape[0])),
        ( np.zeros(q.shape[0]),  np.zeros(q.shape[0]),  np.zeros(q.shape[0]),  np.ones(q.shape[0]))))

    assert rotation_matrices.shape == (4, 4, len(samples))

    # Reshape rotation_matrices
    rotation_matrices = np.einsum('ijt->tij', rotation_matrices)

    assert rotation_matrices.shape == (len(samples), 4, 4)

    return rotation_matrices

def get_transform_matrices(tf_start, tf_end, samples):
    '''
    Gets a sequence of transforms interpolating between tf_start and tf_end

    Args:
        tf_start: a TransformStamped representing the initial transform
        tf_end: a TransformStamped representing the final transform
        samples: a sequence of N numbers between 0 and 1 representing the
                 times at which the interpolation is performed

    Returns:
        a numpy array of shape (N, 4, 4) representing N affine transformations
    '''
    translation_start = np.array([tf_start.transform.translation.x,
                                  tf_start.transform.translation.y,
                                  tf_start.transform.translation.z])
    translation_end = np.array([tf_end.transform.translation.x,
                                  tf_end.transform.translation.y,
                                  tf_end.transform.translation.z])

    orientation_start = np.array([tf_start.transform.rotation.x,
                                  tf_start.transform.rotation.y,
                                  tf_start.transform.rotation.z,
                                  tf_start.transform.rotation.w])
    orientation_end = np.array([tf_end.transform.rotation.x,
                                tf_end.transform.rotation.y,
                                tf_end.transform.rotation.z,
                                tf_end.transform.rotation.w])

    translation_matrices = get_translation_matrices(translation_start,
                                                    translation_end,
                                                    samples)
    rotation_matrices = get_rotation_matrices(orientation_start,
                                              orientation_end,
                                              samples)

    assert translation_matrices.shape == (len(samples), 4, 4)
    assert rotation_matrices.shape == (len(samples), 4, 4)

    # Calculate translation * rotation for every timestep
    transform_matrices = np.einsum('tij,tjk->tik',
                                   translation_matrices,
                                   rotation_matrices)
    return transform_matrices

def get_translation_matrices(translation_start, translation_end, samples):
    '''
    Gets a sequence of transformation matrices representing translations
    interpolated between translation_start and translation_end

    Args:
        translation_start: a sequence of length 3 representing the initial
                           translation
        translation_end: a sequence of length 3 representing the final
                           translation
        samples: a sequence of N numbers between 0 and 1 representing the
                 times at which the interpolation is performed

    Returns:
        a numpy array of shape (N, 4, 4) representing N affine transforms
    '''
    # Interpolate translation vectors
    interp_func = scipy.interpolate.interp1d(
            np.array([0.0, 1.0]),
            np.vstack((translation_start, translation_end)),
            axis=0)
    translations = interp_func(samples)

    # Create identity affine transforms
    translation_matrices = np.tile(np.eye(4), np.array([len(samples), 1, 1]))

    # Set translation components
    translation_matrices[:,0:3,3] = translations

    return translation_matrices

def process_scan(scan, tf_start, tf_end, settings):
    '''
    Do all processing necessary to detect obstacles from a scan and publish
    them
    '''
    ranges = np.array(scan.ranges)
    angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

    # Mask out invalid points
    range_mask = ((ranges > scan.range_min)
                & (ranges < scan.range_max)
                & (ranges >= settings['drone_radius']))

    valid_ranges = ranges[range_mask]
    valid_angles = angles[range_mask]
    samples = np.linspace(0.0, 1.0, len(ranges))[range_mask]

    # Convert ranges to vectors
    valid_range_points = valid_ranges * np.array([np.cos(valid_angles),
                                                  np.sin(valid_angles),
                                                  np.zeros(valid_angles.shape)])
    valid_range_points = np.einsum('it->ti', valid_range_points)
    valid_range_points = np.concatenate(
            (valid_range_points, np.ones((len(valid_ranges), 1))),
            axis=1)

    assert valid_range_points.shape == (len(valid_ranges), 4)

    # Get interpolated transforms
    transform_matrices = get_transform_matrices(tf_start, tf_end, samples)

    assert transform_matrices.shape == (len(valid_ranges), 4, 4)

    # Transform all points into map frame
    transformed_range_points = np.einsum('tij,tj->ti',
                                         transform_matrices,
                                         valid_range_points)

    # Mask out points outside arena bounding box
    mask = ((transformed_range_points[:,0] >= settings['min_x'])
          & (transformed_range_points[:,0] <= settings['max_x'])
          & (transformed_range_points[:,1] >= settings['min_y'])
          & (transformed_range_points[:,1] <= settings['max_y'])
          & (transformed_range_points[:,2] >= settings['min_z'])
          & (transformed_range_points[:,2] <= settings['max_z']))

    # Consider only (x, y) component of points
    points = transformed_range_points[mask][:,:2]

    # Publish empty obstacle list if there are no points
    if not points.size:
        msg = ObstacleArray()
        msg.header.stamp = scan.header.stamp
        obstacle_pub.publish(msg)
        rospy.loginfo('No valid points in laser scan, returning')
        return

    # Group close-together points into clusters
    clusters = [[points[0]]]
    for i in xrange(1, len(points)):
        if (linalg.norm(points[i] - points[i-1])
                < settings['cluster_threshold']):
            clusters[-1].append(points[i])
        else:
            clusters.append([points[i]])

    # If the last cluster is close to the first one, combine them
    if (len(clusters) >= 2
        and linalg.norm(clusters[0][0] - clusters[-1][-1])
                < settings['cluster_threshold']):
        clusters[0].extend(clusters.pop())

    # Estimate positions of clusters
    msg = ObstacleArray()
    msg.header.stamp = scan.header.stamp
    marker_msg = MarkerArray()

    arena_center = np.array((settings['arena_center_x'],
                             settings['arena_center_y']))
    drone_pos = np.einsum('ij,j->i',
                          transform_matrices[0],
                          np.array((0.0, 0.0, 0.0, 1.0)))[:2]

    for cluster in clusters:
        cluster_points = np.stack(cluster, axis=0)
        obst_pos = get_best_circle_for_cluster(cluster_points,
                                               settings['obst_radius'],
                                               drone_pos)

        obst_vel = np.cross(np.concatenate((obst_pos - arena_center,
                                            np.array((0,)))),
                            np.array((0.0, 0.0, 1.0)))
        obst_vel *= settings['obst_speed'] / linalg.norm(obst_vel)

        obst_orientation_angle = np.arctan2(obst_vel[1], obst_vel[0])
        orientation = transformations.quaternion_about_axis(
                obst_orientation_angle,
                (0.0, 0.0, 1.0))

        obst_odom = Odometry()
        obst_odom.header.stamp = scan.header.stamp
        obst_odom.header.frame_id = 'map'
        obst_odom.child_frame_id = 'obstacle{}'.format(str(uuid.uuid1()))
        obst_odom.pose.pose.position.x = obst_pos[0]
        obst_odom.pose.pose.position.y = obst_pos[1]
        obst_odom.pose.pose.position.z = 0.0

        obst_odom.pose.pose.orientation.x = orientation[0]
        obst_odom.pose.pose.orientation.y = orientation[1]
        obst_odom.pose.pose.orientation.z = orientation[2]
        obst_odom.pose.pose.orientation.w = orientation[3]

        obst_odom.twist.twist.linear.x = settings['obst_speed']

        marker = Marker()
        marker.header.stamp = obst_odom.header.stamp
        marker.header.frame_id = 'map'
        marker.ns = 'obstacles'

        global marker_id
        try:
            marker_id += 1
        except NameError:
            marker_id = 0
        marker.id = marker_id

        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(1.0)

        point = Point()
        point.x = obst_pos[0]
        point.y = obst_pos[1]
        marker.points.append(point)

        marker_msg.markers.append(marker)

        obst_msg = Obstacle()
        obst_msg.header.stamp = obst_odom.header.stamp
        obst_msg.odom = obst_odom
        obst_msg.pipe_radius = settings['obst_radius']
        obst_msg.pipe_height = settings['obst_height']

        msg.obstacles.append(obst_msg)

    obstacle_pub.publish(msg)

    if marker_msg.markers:
        marker_pub.publish(marker_msg)

if __name__ == '__main__':
    rospy.init_node('obstacle_detector')
    rospy.Subscriber('scan', LaserScan, scan_callback)

    obstacle_pub = rospy.Publisher('obstacles', ObstacleArray, queue_size=5)
    marker_pub = rospy.Publisher('obstacle_markers', MarkerArray, queue_size=5)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    msg_queue = deque()

    settings = {}
    settings['min_x'] = rospy.get_param('~min_x')
    settings['max_x'] = rospy.get_param('~max_x')
    settings['min_y'] = rospy.get_param('~min_y')
    settings['max_y'] = rospy.get_param('~max_y')
    settings['min_z'] = rospy.get_param('~min_z')
    settings['max_z'] = rospy.get_param('~max_z')
    settings['drone_radius'] = rospy.get_param('~drone_radius')
    settings['cluster_threshold'] = rospy.get_param('~cluster_threshold')
    settings['arena_center_x'] = rospy.get_param('~arena_center_x')
    settings['arena_center_y'] = rospy.get_param('~arena_center_y')
    settings['obst_radius'] = rospy.get_param('~obst_radius')
    settings['obst_height'] = rospy.get_param('~obst_height')
    settings['obst_speed'] = rospy.get_param('~obst_speed')
    settings['max_queue_size'] = rospy.get_param('~max_queue_size')

    timeout = rospy.Duration(rospy.get_param('~timeout'))

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if len(msg_queue) > settings['max_queue_size']:
            rospy.logwarn(
                    'Obstacle detector message queue reached size %d, clearing'
                    % len(msg_queue))
            msg_queue.clear()

        while msg_queue:
            msg = msg_queue.popleft()
            try:
                tf_start = tf_buffer.lookup_transform('map',
                                                      msg.header.frame_id,
                                                      msg.header.stamp,
                                                      timeout)
                tf_end = tf_buffer.lookup_transform(
                        'map',
                        msg.header.frame_id,
                        msg.header.stamp + rospy.Duration(
                            msg.time_increment * (len(msg.ranges) - 1)),
                        timeout)
            except tf2_ros.TransformException as ex:
                rospy.logwarn(ex)
                rospy.logwarn('Clearing obstacle_detector message queue')
                msg_queue.clear()
            else:
                process_scan(msg, tf_start, tf_end, settings)

        rate.sleep()
