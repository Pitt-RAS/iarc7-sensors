/*
 * Filter for altimeter readings
 *
 * This class takes in raw altimeter readings and velocities in the coordinate
 * frame of an altimeter, transforms them into altitudes, and runs a smoothing
 * filter over the data.
 */
#ifndef IARC7_SENSORS_ALTIMETER_FILTER_H_
#define IARC7_SENSORS_ALTIMETER_FILTER_H_

#include <ros/ros.h>
#include <message_filters/subscriber.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <tf2_ros/message_filter.h>
#pragma GCC diagnostic pop

#include <tf2_ros/transform_listener.h>

#include "iarc7_sensors/MovingAverage.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iarc7_msgs/Float64Stamped.h>

namespace iarc7_sensors {

class AltimeterFilter {
  public:
    AltimeterFilter(ros::NodeHandle& nh,
                    const ros::NodeHandle& private_nh,
                    const std::string& altimeter_frame,
                    double altitude_covariance,
                    const std::string& level_quad_frame);
    ~AltimeterFilter() = default;

    void updateFilter(const iarc7_msgs::Float64Stamped& msg);

  private:
    const std::string altimeter_frame_;
    const double altitude_covariance_;
    const std::string level_quad_frame_;

    const ros::Publisher altitude_pose_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    message_filters::Subscriber<iarc7_msgs::Float64Stamped> msg_sub_;
    tf2_ros::MessageFilter<iarc7_msgs::Float64Stamped> msg_filter_;
};

} // namespace iarc7_sensors

#endif // include guard
