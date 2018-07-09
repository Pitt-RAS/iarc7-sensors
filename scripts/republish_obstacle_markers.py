#!/usr/bin/env python2
'''
Republishes obstacles as markers for visualization in RViz

Obstacles observations topic: /obstacles

Thanks Aaron
'''

import rospy
from geometry_msgs.msg import Point, PolygonStamped
from iarc7_msgs.msg import (Obstacle, ObstacleArray)
from visualization_msgs.msg import Marker

def callback(msg):
    marker = Marker()
    marker.header = msg.header
    marker.header.frame_id = "map"
    marker.ns = 'obstacles'
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.MODIFY

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1

    marker.lifetime = rospy.Duration(2)
    marker.frame_locked = False

    for obstacle in msg.obstacles:
        point = Point()
        point.x = obstacle.odom.pose.pose.position.x
        point.y = obstacle.odom.pose.pose.position.y
        point.z = 0
        marker.points.append(point)

    vis_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('republish_obstacle_detections')
   
    vis_pub = rospy.Publisher('/obstacle_detection_markers',
                              Marker,
                              queue_size=5)
    rospy.Subscriber('/obstacles', ObstacleArray, callback)
    rospy.spin()

