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

AltimeterFilter::AltimeterFilter(const ros::NodeHandle& node_handle,
                                 const std::string& altimeter_frame,
                                 const std::string& level_quad_frame)
      : node_handle_(node_handle), tf_listener_(tf_buffer_),
        altimeter_frame_(altimeter_frame), level_quad_frame_(level_quad_frame),
        filter_(ros::Duration(getParam(node_handle_,
                                       "filter_time_constant",
                                       0.0))) {
}

void AltimeterFilter::updateFilter(double altitude,
                                   double velocity,
                                   ros::Time time) {

    geometry_msgs::PointStamped altimeter_frame_msg;
    altimeter_frame_msg.header.stamp = time;
    altimeter_frame_msg.header.frame_id = altimeter_frame_;
    altimeter_frame_msg.point.x = altitude;

    geometry_msgs::PointStamped level_frame_msg;

    try {
        tf_buffer_.transform(altimeter_frame_msg,
                             level_frame_msg,
                             level_quad_frame_,
                             ros::Duration(0.5));
    } catch (const std::exception& ex) {
        ROS_ERROR("Exception looking up transform in AltimeterFilter: %s",
                  ex.what());
        return;
    }

    double transformed_velocity = velocity * (-level_frame_msg.point.z) / altitude;

    filter_.updateFilter(-level_frame_msg.point.z, transformed_velocity, time);
}

double AltimeterFilter::getFilteredAltitude(ros::Time time) const {
    return filter_.getFilteredValue(time);
}

} // namespace iarc7_sensors
