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
#include <sensor_msgs/Range.h>

namespace iarc7_sensors {

class AltimeterFilter {
  public:
    /// @param[in] altitude_variance_func  {Should return the variance for a given
    ///                                     altitude measurement}
    AltimeterFilter(ros::NodeHandle& nh,
                    const std::string& altimeter_frame,
                    std::function<double(double)> altitude_variance_func,
                    const std::string& level_quad_frame,
                    double max_altitude=std::numeric_limits<double>::max());
    ~AltimeterFilter() = default;

    void updateFilter(const sensor_msgs::Range& msg);

  private:
    const std::string altimeter_frame_;
    const std::function<double(double)> altitude_variance_func_;
    const std::string level_quad_frame_;

    const ros::Publisher altitude_pose_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    message_filters::Subscriber<sensor_msgs::Range> msg_sub_;
    tf2_ros::MessageFilter<sensor_msgs::Range> msg_filter_;

    const double max_altitude_;
};

} // namespace iarc7_sensors

#endif // include guard
