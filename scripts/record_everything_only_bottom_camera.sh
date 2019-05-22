#!/usr/bin/env sh
rosbag record --all -x "/bottom_camera(.*)|/iarc7_vision_node(.*)|/optical_flow_estimator(.*)" /bottom_camera/rgb/image_raw
