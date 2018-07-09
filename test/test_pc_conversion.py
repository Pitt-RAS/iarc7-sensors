#!/usr/bin/env python2

import rospy
from tf import transformations
import tf2_ros
from image_geometry import PinholeCameraModel

import math
import numpy as np
from numpy.linalg import inv
import ros_numpy
from ros_numpy import numpy_msg

from cv_bridge import CvBridge, CvBridgeError

import sklearn
from sklearn.cluster import DBSCAN

from iarc7_msgs.msg import Obstacle, ObstacleArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

camera = PinholeCameraModel()
bridge = CvBridge()


def process_depth_callback(data):

    try:
        image = bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
	print(e)

    depth = np.asarray(image)

    good_indices = np.where(depth > 1)

    row_coords = good_indices[0]
    column_coords = good_indices[1]

    random_mask = np.random.choice([False, True], len(row_coords), p = [0.90, 0.10])

    row_coords = row_coords[random_mask]
    column_coords = column_coords[random_mask]

    masked_depth = depth[row_coords, column_coords]

    xyz_array = np.zeros((len(row_coords), 3))

    xyz_array[:,0] = (column_coords- camera.cx())/(camera.fx()*1000.0)
    xyz_array[:,1] = (row_coords - camera.cy())/(camera.fy()*1000.0)

    xyz_array[:,0] = np.multiply(xyz_array[:,0], masked_depth)
    xyz_array[:,1] = np.multiply(xyz_array[:,1], masked_depth)

    xyz_array[:,2] = masked_depth/1000.0


    array_for_conversion = np.zeros((len(row_coords),), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('r', np.uint8),
        ('g', np.uint8),
        ('b', np.uint8)])

    array_for_conversion['x'] = xyz_array[:,0]
    array_for_conversion['y'] = xyz_array[:,1]
    array_for_conversion['z'] = xyz_array[:,2]
 
    #print(xyz_array)
    #print(xyz_array.shape)
    #print(xyz_array.dtype)

    xyz_array = np.transpose(xyz_array)

    cloud_msg = ros_numpy.msgify(numpy_msg(PointCloud2), array_for_conversion)

    print(data.header)

    cloud_msg.header = data.header

    pc_pub.publish(cloud_msg)

def get_calibration_parameters(parameters):

    print("Got the params")

    camera.fromCameraInfo(parameters)
    camera_info_sub.unregister()


if __name__ == '__main__':

    rospy.init_node('pc_tester_r200')

    rospy.Subscriber('/camera/depth/image_raw', Image, process_depth_callback, queue_size = 1)

    camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, get_calibration_parameters)
    
    pc_pub = rospy.Publisher('/obst_points', PointCloud2, queue_size=5)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

        rate.sleep()

