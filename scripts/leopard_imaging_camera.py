#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
from m021v4l2 import Capture1280x720
import numpy as np
import rospy
#from dynamic_reconfigure.server import Server
#from iarc7_sensors.cfg import LeopardImagingCameraConfig

from iarc7_safety.SafetyClient import SafetyClient
from iarc7_safety.iarc_safety_exception import IARCFatalSafetyException

from sensor_msgs.msg import Image

rospy.init_node('leopard_imaging_camera')
bridge = CvBridge()
pub = rospy.Publisher('/bottom_camera/rgb/image_raw', Image, queue_size=5)

#cap = None

#def dynamic_reconfigure_callback(config, level):
#    global cap
#    rospy.logerr(config)
#    cap = Capture1280x720(
#            '/dev/v4l/by-id/usb-Leopard_Imaging_MT9M021C_0000000003-video-index0',
#            config['r_gain'],
#            config['g_gain'],
#            config['b_gain'])
#    rospy.logerr(cap)
#    return config
#dynamic_reconfigure_server = Server(LeopardImagingCameraConfig,
#                                    dynamic_reconfigure_callback)

cap = Capture1280x720(
        '/dev/v4l/by-id/usb-Leopard_Imaging_MT9M021C_0000000003-video-index0',
        0,
        0,
        0)

rate = rospy.Rate(30)

#while cap is None and not rospy.is_shutdown():
#    rate.sleep()

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

safety_client = SafetyClient('leopard_imaging_camera')
if not safety_client.form_bond():
    raise IARCFatalSafetyException('Leopard imaging camera could not form bond'
            + ' with safety monitor')

while not rospy.is_shutdown():
    if safety_client.is_fatal_active():
        raise IARCFatalSafetyException(
                'Fatal active in Leopard Imaging camera node')

    ret, frame = cap.read()
    stamp = rospy.Time.now()
    if ret:
        cv_image = bridge.cv2_to_imgmsg(frame, 'bgr8')
        cv_image.header.stamp = stamp
        cv_image.header.frame_id = 'bottom_camera_rgb_optical_frame'
        pub.publish(cv_image)
    else:
        raise IARCFatalSafetyException('No frame from camera')
    rate.sleep()
