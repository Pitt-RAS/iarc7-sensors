#!/usr/bin/env python
import rospy

from iarc7_msgs.msg import Obstacle, ObstacleArray

from visualization_msgs.msg import Marker, MarkerArray

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('fake_obstacles')
    obstacle_publisher = rospy.Publisher('obstacles', ObstacleArray, queue_size=0)
    marker_pub = rospy.Publisher('obstacle_markers', MarkerArray, queue_size=5)

    while not rospy.is_shutdown():
        all_ob = ObstacleArray()

        all_ob.header.stamp = rospy.Time.now()
        all_ob.header.frame_id = 'map'

        obst = Obstacle()
        obst.base_height = 0.0
        obst.base_radius = 0.0
        obst.pipe_height = 4.0
        obst.pipe_radius = .1

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'map'

        odom.child_frame_id = 'obstacle_fake_frame'

        odom.pose.pose.position.x = 2.0
        odom.pose.pose.position.y = 2.0

        obst.odom = odom

        all_ob.obstacles = [obst]


        marker_msg = MarkerArray()

        marker = Marker()
        marker.header.stamp = odom.header.stamp
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
        point.x = .5
        point.y = .5
        marker.points.append(point)

        marker_msg.markers.append(marker)

        obstacle_publisher.publish(all_ob)

        if marker_msg.markers:
            marker_pub.publish(marker_msg)


        rospy.sleep(0.07)


