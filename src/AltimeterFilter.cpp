/*
 * Filter for altimeter readings
 *
 * This class takes in raw altimeter readings and velocities in the coordinate
 * frame of an altimeter, transforms them into altitudes, and runs a smoothing
 * filter over the data.
 */
#include "iarc7_sensors/AltimeterFilter.h"

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace iarc7_sensors {

AltimeterFilter::AltimeterFilter(const ros::NodeHandle& node_handle,
                                 const std::string& altimeter_frame,
                                 const std::string& level_quad_frame)
      : node_handle_(node_handle), tf_listener_(tf_buffer_),
        altimeter_frame_(altimeter_frame), level_quad_frame_(level_quad_frame) {
    double filter_time_secs;
    node_handle_.param("filter_time_constant", filter_time_secs, 0.0);
    filter_time_constant_ = ros::Duration(filter_time_secs);
}

void AltimeterFilter::updateFilter(double altitude, double velocity, ros::Time time) {
    if (time <= altitude_buffer_.back().stamp) {
        ROS_ERROR("Tried to add message to altimeter filter with timestamp older than newest message in filter");
        return;
    }

    geometry_msgs::PointStamped altimeter_frame_msg;
    altimeter_frame_msg.header.stamp = time;
    altimeter_frame_msg.header.frame_id = altimeter_frame_;
    altimeter_frame_msg.point.z = altitude;

    geometry_msgs::PointStamped level_frame_msg;
    try {
        tf_buffer_.transform(altimeter_frame_msg, level_frame_msg, level_quad_frame_);
    } catch (const std::exception& ex) {
        ROS_ERROR("Exception looking up transform in AltimeterFilter: %s", ex.what());
        return;
    }

    altitude_buffer_.emplace_back(-level_frame_msg.point.z,
                                  level_frame_msg.header.stamp);

    ros::Duration time_since_second_reading =
        altitude_buffer_.back().stamp - altitude_buffer_[1].stamp;
    if (time_since_second_reading >= filter_time_constant_ && altitude_buffer_.size() > 1) {
        altitude_buffer_.pop_front();
    }
}

double AltimeterFilter::getFilteredAltitude(ros::Time time) const {
    if (altitude_buffer_.empty()) {
        ROS_ERROR("Attempt to get altitude from empty filter");
        return std::numeric_limits<double>::quiet_NaN();
    }

    if (time - altitude_buffer_.back().stamp > filter_time_constant_) {
        ROS_WARN("Attempt to get altitude from filter when newest measurement is older than the filter time");
        return altitude_buffer_.back().altitude;
    }

    if (time < altitude_buffer_.back().stamp) {
        ROS_WARN("Attempt to get altitude at time before the most recent filter update");
        return altitude_buffer_.back().altitude;
    }

    if (altitude_buffer_.size() == 1) {
        return altitude_buffer_.back().altitude;
    }

    double altitude_sum = 0;

    // handle first item
    size_t i;
    for (i = 0; i < altitude_buffer_.size(); i++) {
        if (altitude_buffer_[i].stamp > time - filter_time_constant_) {
            double time_for_next_interval = (altitude_buffer_[i+1].stamp
                                             - altitude_buffer_[i].stamp).toSec();
            altitude_sum += altitude_buffer_[i].altitude * time_for_next_interval / 2;

#pragma GCC warning "TODO: This chunk of math isn't readable at all, needs better variable names"
            ros::Duration dt = altitude_buffer_[i].stamp - altitude_buffer_[i-1].stamp;
            double dt2 = (altitude_buffer_[i].stamp - time).toSec() / 2.0;
            ros::Time interp_point = time + ros::Duration(dt2);
            double t2 = (altitude_buffer_[i].stamp - interp_point).toSec() / dt.toSec();
            double t = (interp_point - altitude_buffer_[i-1].stamp).toSec() / dt.toSec();
            altitude_sum += altitude_buffer_[i-1].altitude * t * dt2;
            altitude_sum += altitude_buffer_[i].altitude * t2 * dt2;

            break;
        }
    }

    // handle middle items
    i++;
    for (; i < altitude_buffer_.size() - 1; i++) {
        ros::Duration duration = altitude_buffer_[i+1].stamp
            - altitude_buffer_[i-1].stamp;
        altitude_sum += altitude_buffer_[i].altitude * duration.toSec() / 2;
    }

    // handle last item
    double last_time = altitude_buffer_.back().stamp.toSec();
    double next_to_last_time = altitude_buffer_[altitude_buffer_.size() - 2].stamp.toSec();
    double last_item_weight = time.toSec() - (last_time + next_to_last_time) / 2;
    altitude_sum += altitude_buffer_.back().altitude * last_item_weight;

    return altitude_sum / filter_time_constant_.toSec();
}

} // namespace iarc7_sensors
