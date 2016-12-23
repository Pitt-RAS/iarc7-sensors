/*
 * Moving Average Filter (linear FIR filter with equal weights)
 */
#ifndef IARC7_SENSORS_MOVING_AVERAGE_H_
#define IARC7_SENSORS_MOVING_AVERAGE_H_

#include <ros/ros.h>

#include <deque>

namespace iarc7_sensors {

class MovingAverage {
  public:
    MovingAverage(const ros::Duration& time = ros::Duration(0));
    ~MovingAverage() = default;

    void updateFilter(double value, double derivative, const ros::Time& time);
    double getFilteredValue(const ros::Time& time) const;

  private:
    struct MovingAverageDataPoint {
        double value;
        double derivative;
        ros::Time stamp;

        MovingAverageDataPoint(double value,
                               double derivative,
                               const ros::Time& stamp)
            : value(value), derivative(derivative), stamp(stamp) {}
    };

    const ros::Duration filter_time_constant_;
    std::deque<MovingAverageDataPoint> value_buffer_;
};

} // namespace iarc7_sensors

#endif // include guard
