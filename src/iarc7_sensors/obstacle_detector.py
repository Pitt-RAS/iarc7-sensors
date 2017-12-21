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

def slerp(q0, q1, t):
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

def get_circle(points, radius, drone_pos, settings):
    assert points.size

    mean_point = np.mean(points, axis=0)
    guess_pos_offset = mean_point - drone_pos
    guess_pos_offset *= (
            (linalg.norm(guess_pos_offset) + settings['obst_radius'])
          / linalg.norm(guess_pos_offset)
        )
    guess_pos = drone_pos + guess_pos_offset

    if points.shape[0] == 1:
        return guess_pos
    else:
        def f(x):
            return linalg.norm(x - points, axis=1) - radius

        def Dfun(x):
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
    assert orientation_start.shape == (4,)
    assert orientation_end.shape == (4,)

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

    rotation_matrices = np.einsum('ijt->tij', rotation_matrices)

    assert rotation_matrices.shape == (len(samples), 4, 4)

    return rotation_matrices

def get_transform_matrices(tf_start, tf_end, samples):
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

    transform_matrices = np.einsum('tij,tjk->tik',
                                   translation_matrices,
                                   rotation_matrices)
    return transform_matrices

def get_translation_matrices(translation_start, translation_end, samples):
    interp_func = scipy.interpolate.interp1d(
            np.array([0.0, 1.0]),
            np.vstack((translation_start, translation_end)),
            axis=0)
    translations = interp_func(samples)

    translation_matrices = np.tile(np.eye(4), np.array([len(samples), 1, 1]))
    translation_matrices[:,0:3,3] = translations

    return translation_matrices

def process_scan(scan, tf_start, tf_end, settings):
    ranges = np.array(scan.ranges)
    angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

    range_mask = ((ranges > scan.range_min)
                & (ranges < scan.range_max)
                & (ranges >= settings['drone_radius']))

    valid_ranges = ranges[range_mask]
    valid_angles = angles[range_mask]
    samples = np.linspace(0.0, 1.0, len(ranges))[range_mask]

    valid_range_points = valid_ranges * np.array([np.cos(valid_angles),
                                                  np.sin(valid_angles),
                                                  np.zeros(valid_angles.shape)])
    valid_range_points = np.einsum('it->ti', valid_range_points)
    valid_range_points = np.concatenate(
            (valid_range_points, np.ones((len(valid_ranges), 1))),
            axis=1)

    assert valid_range_points.shape == (len(valid_ranges), 4)

    transform_matrices = get_transform_matrices(tf_start, tf_end, samples)

    assert transform_matrices.shape == (len(valid_ranges), 4, 4)

    transformed_range_points = np.einsum('tij,tj->ti',
                                         transform_matrices,
                                         valid_range_points)

    mask = ((transformed_range_points[:,0] >= settings['min_x'])
          & (transformed_range_points[:,0] <= settings['max_x'])
          & (transformed_range_points[:,1] >= settings['min_y'])
          & (transformed_range_points[:,1] <= settings['max_y'])
          & (transformed_range_points[:,2] >= settings['min_z'])
          & (transformed_range_points[:,2] <= settings['max_z']))

    points = transformed_range_points[mask][:,:2]

    if not points.size:
        msg = ObstacleArray()
        msg.header.stamp = scan.header.stamp
        obstacle_pub.publish(msg)
        rospy.loginfo('No valid points in laser scan, returning')
        return

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
        obst_pos = get_circle(cluster_points,
                              settings['obst_radius'],
                              drone_pos,
                              settings)

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
