#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
from m021v4l2 import Capture800x460
import rospy

from sensor_msgs.msg import Image

rospy.init_node('leopard_imaging_camera')
bridge = CvBridge()
pub = rospy.Publisher('/bottom_camera/image_raw', Image, queue_size=5)

cap = Capture800x460('/dev/v4l/by-id/usb-Leopard_Imaging_MT9M021C_0000000003-video-index0')

rate = rospy.Rate(90)
while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret:
        cv_image = bridge.cv2_to_imgmsg(frame, 'bgr8')
        pub.publish(cv_image)
    rate.sleep()
