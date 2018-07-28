#!/usr/bin/env python2

import math
import numpy as np
from numpy.linalg import inv

import rospy
from image_geometry import PinholeCameraModel
from tf import transformations
import tf2_ros

from cv_bridge import CvBridge, CvBridgeError

import sklearn
from sklearn.cluster import DBSCAN

from iarc7_msgs.msg import Obstacle, ObstacleArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2, PointField
import message_filters

camera = PinholeCameraModel()
bridge = CvBridge()

def xyz_array_to_pointcloud2(points, stamp=None, frame_id=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12*points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg

def process_depth_callback(data, camera_info):
    global earliest_allowed_time
    if earliest_allowed_time is None:
        trans = tf_buffer.lookup_transform(data.header.frame_id,
                                           'map',
                                           rospy.Time(0),
                                           rospy.Duration(10.0))
        earliest_allowed_time = trans.header.stamp

    if data.header.stamp < earliest_allowed_time:
        rospy.logerr('Received frame with stamp %.3f before earliest allowed %.3f',
                data.header.stamp.to_sec(),
                earliest_allowed_time.to_sec())
        return

    camera.fromCameraInfo(camera_info)

    # m_to_o = map_to_optical
    m_to_o_trans = tf_buffer.lookup_transform(data.header.frame_id, "map", data.header.stamp, rospy.Duration(1))

    o_to_m_trans = tf_buffer.lookup_transform("map", data.header.frame_id, data.header.stamp, rospy.Duration(1))

    m_to_o_quat = np.array([m_to_o_trans.transform.rotation.x,
                           m_to_o_trans.transform.rotation.y,
                           m_to_o_trans.transform.rotation.z,
                           m_to_o_trans.transform.rotation.w])

    o_to_m_quat = np.array([o_to_m_trans.transform.rotation.x,
                           o_to_m_trans.transform.rotation.y,
                           o_to_m_trans.transform.rotation.z,
                           o_to_m_trans.transform.rotation.w])

    m_to_o_rot_matrix = transformations.quaternion_matrix(m_to_o_quat)
    o_to_m_rot_matrix = transformations.quaternion_matrix(o_to_m_quat)
    o_to_m_rot_matrix = o_to_m_rot_matrix[:3,:3]
    m_to_o_rot_matrix = m_to_o_rot_matrix[:3,:3]

    unit_x = np.array([1, 0, 0])
    unit_y = np.array([0, 1, 0])
    unit_z = np.array([0, 0, 1])

    # The representation of unit x,y,z vectors in the map frame
    # within the obstacle frame
    transformed_x = np.matmul(m_to_o_rot_matrix, unit_x)
    transformed_y = np.matmul(m_to_o_rot_matrix, unit_y)
    transformed_z = np.matmul(m_to_o_rot_matrix, unit_z)

    # The translation from the center of the map to the center of the optical frame
    m_to_o_translation = np.array([o_to_m_trans.transform.translation.x,
                       o_to_m_trans.transform.translation.y,
                       o_to_m_trans.transform.translation.z])

    try:
        image = bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
        print(e)

    subsample_factor = 3
    depth = np.asarray(image)[::subsample_factor,::subsample_factor]

    # pixels without depth information have a pixel value of 0, so we 
    # ignore all of those
    # Also throw out points that are far away
    good_indices = np.where((depth > 0.6 * 1000) & (depth < 5 * 1000))

    row_coords = good_indices[0]
    column_coords = good_indices[1]
    # Convert depth values from millimeters to meters
    depth = depth[row_coords, column_coords] / 1000.0

    obstacle_points = np.zeros((len(row_coords), 3))

    assert obstacle_points.shape[-1] == 3

    # Transform the depth values at each pixel into a 3d vector in the optical frame
    obstacle_points[:,0] = (subsample_factor*column_coords - camera.cx())/(camera.fx())
    obstacle_points[:,1] = (subsample_factor*row_coords - camera.cy())/(camera.fy())

    obstacle_points[:,0] = np.multiply(obstacle_points[:,0], depth)
    obstacle_points[:,1] = np.multiply(obstacle_points[:,1], depth)

    obstacle_points[:,2] = depth

    obstacle_points = obstacle_points.astype(np.float32)

    # Throw out points on the floor
    z_scalars = np.tensordot(obstacle_points, transformed_z, axes = 1)
    obstacle_points = obstacle_points[z_scalars + m_to_o_translation[2] >= np.maximum(0.2, 0.1 * depth),:]

    # Throw out points that are far away
    #depth_mask = obstacle_points[:,2] < 5.0
    #obstacle_points = obstacle_points[depth_mask,:]

    # Enforce a upper bound on the number of points going into dbscan to prevent the CPU 
    # from getting annihilated
    MAX_POINTS = 400.
    prob_true = MAX_POINTS/len(obstacle_points) if len(obstacle_points) > MAX_POINTS else 1
    random_mask = np.random.choice([True, False],
                                   len(obstacle_points),
                                   p = [prob_true, 1-prob_true])

    sampled_obstacle_points = obstacle_points[random_mask]

    debug_sampled_points = True
    if debug_sampled_points:
        pointcloud = xyz_array_to_pointcloud2(
                sampled_obstacle_points, data.header.stamp, data.header.frame_id)
        pointcloud_pub.publish(pointcloud)

    # eps - min distance between clusters to be considered separate clusters (currently about 
    # the radius of a roomba) 
    # The manhattan metric is chosen because it is computationally cheaper than calculating the euclidean
    # distance and is just as accurate (in terms of the output clusters).
    if sampled_obstacle_points.shape[0] == 0:
        return
    if 'right' in data.header.frame_id:
        min_samples = 20
    elif 'left':
        min_samples = 80
    else:
        min_samples = 20
    db = DBSCAN(min_samples=min_samples,
                algorithm='ball_tree',
                eps = .15,
                leaf_size = 20,
                metric = 'manhattan').fit(sampled_obstacle_points)

    # labels is merely an array of integers from 0 to the number of clusters, 
    # specifying which points from dbscan_input belong to which cluster
    labels = db.labels_

    # The noisy labels are labeled with -1, so those are unaccounted for
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

    # The final output
    obstacles = ObstacleArray()

    for i in range(0, n_clusters):

        obstacle = Obstacle()

        mask = np.where(labels == i)
        masks = np.array_split(mask[0], 3)
        top_third, middle_third, bottom_third = masks
        if not len(top_third) or not len(middle_third) or not len(bottom_third):
            continue
        
        # Isolate the vectors pointing to the top third of the obstacle
        top_third_vectors = sampled_obstacle_points[top_third,:]
        z_scalars = np.zeros(top_third.size)
        full_vector = np.zeros((3, top_third.size))

        before_loop = rospy.get_time()

        z_scalars = np.tensordot(top_third_vectors, transformed_z, axes = 1)
    
        # The point in the point cloud that had the highest z coordinate in the map frame 
        # will have the highest z scalar
        max_z = np.argmax(z_scalars)

        # Transform the vector to the top of the obstacle from the optical frame to 
        # the map frame
        map_vector_to_obs = np.matmul(o_to_m_rot_matrix, top_third_vectors[max_z,:])

        obstacle.pipe_height = map_vector_to_obs[2] + m_to_o_translation[2]

        # The height of a roomba is about a tenth of a meter. Any cluster with a height
        # lower than that is definitely not a roomba
        if obstacle.pipe_height < 0.1:
            continue

        bottom_third_vector = sampled_obstacle_points[bottom_third,:]
        full_vector = np.zeros([3, bottom_third.size])
        x_scalars = np.zeros([bottom_third.size])
        y_scalars = np.zeros([bottom_third.size])
 
        y_scalars = np.tensordot(bottom_third_vector, transformed_y, axes = 1)
        x_scalars = np.tensordot(bottom_third_vector, transformed_x, axes = 1)
        
        x_min = np.argmin(x_scalars)
        y_min = np.argmin(y_scalars)
        x_max = np.argmax(x_scalars)
        y_max = np.argmax(y_scalars)

        camera_vector_to_center = bottom_third_vector[x_min,:] + bottom_third_vector[x_max,:] + \
                                  bottom_third_vector[y_min,:] + bottom_third_vector[y_max,:]
        camera_vector_to_center = camera_vector_to_center/4

        # Now transfer camera_vector_to_center to the map frame
        map_vector_to_base_center = np.matmul(o_to_m_rot_matrix, camera_vector_to_center)

        map_coordinates = map_vector_to_base_center + m_to_o_translation

        obstacle.odom.header.stamp = data.header.stamp
        obstacle.odom.header.frame_id = 'map'
        obstacle.odom.header.frame_id = 'DEADBEEF'

        obstacle.odom.pose.pose.position.x = map_coordinates[0]
        obstacle.odom.pose.pose.position.y = map_coordinates[1]
        obstacle.odom.pose.pose.position.z = 0

        obstacle.odom.pose.covariance[0] = 0.4
        obstacle.odom.pose.covariance[1] = 0.0
        obstacle.odom.pose.covariance[6] = 0.0
        obstacle.odom.pose.covariance[7] = 0.4

        obstacle.base_height = 0.1
        obstacle.base_radius = 0.15
        obstacle.pipe_radius = 0.05

        obstacle.header.stamp = data.header.stamp
        obstacle.header.frame_id = 'map'
        obstacles.obstacles.append(obstacle)

    obstacles.header.stamp = data.header.stamp
    obstacles.header.frame_id = 'map'

    obstacle_pub.publish(obstacles)

def get_calibration_parameters(parameters):
    camera.fromCameraInfo(parameters)
    camera_info_sub.unregister()

if __name__ == '__main__':
    rospy.init_node('obstacle_detector_r200')

    image_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
    camera_info_sub = message_filters.Subscriber("/camera/depth/camera_info", CameraInfo)

    obstacle_pub = rospy.Publisher('/detected_obstacles', ObstacleArray, queue_size=5)
    pointcloud_pub = rospy.Publisher('~clustering_pointcloud', PointCloud2, queue_size=1)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    earliest_allowed_time = None

    ts = message_filters.TimeSynchronizer([image_sub, camera_info_sub], 10)
    ts.registerCallback(process_depth_callback)

    rospy.spin()
