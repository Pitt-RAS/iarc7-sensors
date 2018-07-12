#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
from m021v4l2 import Capture1280x720
import numpy as np
import rospy

from sensor_msgs.msg import Image

rospy.init_node('leopard_imaging_daemon')
bridge = CvBridge()
pub = rospy.Publisher('/bottom_camera/rgb/image_raw', Image, queue_size=5)

cap = Capture1280x720(
        '/dev/v4l/by-id/usb-Leopard_Imaging_MT9M021C_0000000003-video-index0',
        0,
        0,
        0)

rate = rospy.Rate(30)

while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time():
    rate.sleep()

if rospy.is_shutdown():
    raise rospy.ROSInterruptException()

ret = False
start_time = rospy.Time.now()
while (not ret
       and not rospy.is_shutdown()
       and (rospy.Time.now() - start_time) < rospy.Duration(5.0)):
    ret, _ = cap.read()
    rate.sleep()

if rospy.is_shutdown():
    raise rospy.ROSInterruptException()

assert ret

while not rospy.is_shutdown():
    ret, frame = cap.read()
    assert ret

    CAMERA_TIMESTAMP_OFFSET = -0.050

    stamp = rospy.Time.now() + rospy.Duration(CAMERA_TIMESTAMP_OFFSET)
    cv_image = bridge.cv2_to_imgmsg(frame, 'bgr8')
    cv_image.header.stamp = stamp
    cv_image.header.frame_id = 'bottom_camera_rgb_optical_frame'
    pub.publish(cv_image)

    rate.sleep()
