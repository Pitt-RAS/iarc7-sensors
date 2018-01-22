/*
 * Filter for altimeter readings
 *
 * This class takes in raw altimeter readings and velocities in the coordinate
 * frame of an altimeter, transforms them into altitudes, and runs a smoothing
 * filter over the data.
 */
#include "iarc7_sensors/AltimeterFilter.hpp"

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace iarc7_sensors {

AltimeterFilter::AltimeterFilter(ros::NodeHandle& nh,
                                 const std::string& altimeter_frame,
                                 std::function<double(double)> altitude_variance_func,
                                 const std::string& level_quad_frame)
      : altimeter_frame_(altimeter_frame),
        altitude_variance_func_(std::move(altitude_variance_func)),
        level_quad_frame_(level_quad_frame),
        altitude_pose_pub_(
            nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                "altimeter_pose", 0)),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        msg_sub_(nh, "altimeter_reading", 100),
        msg_filter_(msg_sub_, tf_buffer_, level_quad_frame, 100, nullptr)
{
    msg_filter_.registerCallback(&AltimeterFilter::updateFilter, this);
}

void AltimeterFilter::updateFilter(const sensor_msgs::Range& msg)
{
    if (!std::isfinite(msg.range)
     || msg.range < msg.min_range
     || msg.range > msg.max_range) {
        ROS_DEBUG("AltimeterFilter skipped out-of-range reading (%f)", msg.range);
        return;
    }

    ros::Time time = msg.header.stamp;

    geometry_msgs::PointStamped altimeter_frame_msg;
    altimeter_frame_msg.header.stamp = time;
    altimeter_frame_msg.header.frame_id = altimeter_frame_;
    altimeter_frame_msg.point.x = msg.range;

    geometry_msgs::PointStamped level_frame_msg;

    try {
        tf_buffer_.transform(altimeter_frame_msg, level_frame_msg, level_quad_frame_);
    } catch (const std::exception& ex) {
        ROS_ERROR("Exception looking up transform in AltimeterFilter: %s",
                  ex.what());
        return;
    }

    // Publish pose estimate
    geometry_msgs::PoseWithCovarianceStamped altimeter_pose_msg;
    altimeter_pose_msg.header.frame_id = "map";
    altimeter_pose_msg.header.stamp = time;
    altimeter_pose_msg.pose.pose.position.z = -level_frame_msg.point.z;

    // This is a 6x6 matrix and z height is variable 2,
    // so the z height covariance is at location (2,2)
    altimeter_pose_msg.pose.covariance[2*6 + 2] =
        altitude_variance_func_(altimeter_pose_msg.pose.pose.position.z);

    // Check if the filter spit out a valid estimate
    if (!std::isfinite(altimeter_pose_msg.pose.pose.position.z)) {
        ROS_ERROR("Altimeter filter returned invalid altitude %f",
                  altimeter_pose_msg.pose.pose.position.z);
    } else {
        // Publish the transform
        altitude_pose_pub_.publish(altimeter_pose_msg);
    }
}

} // namespace iarc7_sensors
