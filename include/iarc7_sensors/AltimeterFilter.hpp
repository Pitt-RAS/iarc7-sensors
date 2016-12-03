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
#include <tf2_ros/transform_listener.h>

#include "iarc7_sensors/MovingAverage.hpp"

namespace iarc7_sensors {

class AltimeterFilter {
  public:
    AltimeterFilter(const ros::NodeHandle& node_handle,
                    const std::string& altimeter_frame,
                    const std::string& level_quad_frame);
    ~AltimeterFilter() = default;

    void updateFilter(double altitude, double velocity, ros::Time time);
    double getFilteredAltitude(ros::Time time) const;

  private:
    ros::NodeHandle node_handle_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string altimeter_frame_;
    std::string level_quad_frame_;

    iarc7_sensors::MovingAverage filter_;
};

} // namespace iarc7_sensors

#endif // include guard
