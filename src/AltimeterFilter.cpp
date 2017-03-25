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

template<typename T>
T getParam(const ros::NodeHandle& nh, const std::string& name, const T& def) {
    T val;
    nh.param(name, val, def);
    return val;
}

namespace iarc7_sensors {

AltimeterFilter::AltimeterFilter(ros::NodeHandle& node_handle,
                                 const std::string& altimeter_frame,
                                 double altitude_covariance,
                                 const std::string& level_quad_frame)
      : altimeter_frame_(altimeter_frame),
        altitude_covariance_(altitude_covariance),
        level_quad_frame_(level_quad_frame),
        altitude_pose_pub_(
            node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                "altimeter_pose", 0)),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        msg_sub_(node_handle, "altimeter_reading", 100),
        msg_filter_(msg_sub_, tf_buffer_, level_quad_frame, 100, nullptr),
        filter_(ros::Duration(getParam(node_handle,
                                       "filter_time_constant",
                                       0.0)))
{
    msg_filter_.registerCallback(&AltimeterFilter::updateFilter, this);
}

void AltimeterFilter::updateFilter(const iarc7_msgs::Float64Stamped& msg)
{
    ros::Time time = msg.header.stamp;

    geometry_msgs::PointStamped altimeter_frame_msg;
    altimeter_frame_msg.header.stamp = time;
    altimeter_frame_msg.header.frame_id = altimeter_frame_;
    altimeter_frame_msg.point.x = msg.data;

    geometry_msgs::PointStamped level_frame_msg;

    try {
        tf_buffer_.transform(altimeter_frame_msg, level_frame_msg, level_quad_frame_);
    } catch (const std::exception& ex) {
        ROS_ERROR("Exception looking up transform in AltimeterFilter: %s",
                  ex.what());
        return;
    }

    filter_.updateFilter(-level_frame_msg.point.z, 0.0, time);

    // Publish pose estimate
    geometry_msgs::PoseWithCovarianceStamped altimeter_pose_msg;
    altimeter_pose_msg.header.frame_id = "map";
    altimeter_pose_msg.header.stamp = time;
    altimeter_pose_msg.pose.pose.position.z = getFilteredAltitude(time);

    // This is a 6x6 matrix and z height is variable 2,
    // so the z height covariance is at location (2,2)
    altimeter_pose_msg.pose.covariance[2*6 + 2] = altitude_covariance_ * (1 + 8*std::exp(-4*altimeter_pose_msg.pose.pose.position.z));

    // Check if the filter spit out a valid estimate
    if (!std::isfinite(altimeter_pose_msg.pose.pose.position.z)) {
        ROS_ERROR("Altimeter filter returned invalid altitude %f",
                  altimeter_pose_msg.pose.pose.position.z);
    } else {
        // Publish the transform
        altitude_pose_pub_.publish(altimeter_pose_msg);
    }
}

double AltimeterFilter::getFilteredAltitude(ros::Time time) const
{
    return filter_.getFilteredValue(time);
}

} // namespace iarc7_sensors
