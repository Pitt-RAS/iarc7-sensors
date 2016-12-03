/*
 * Moving Average Filter (linear FIR filter with equal weights)
 */
#include "iarc7_sensors/MovingAverage.hpp"

#include <ros/ros.h>

namespace iarc7_sensors {

MovingAverage::MovingAverage(const ros::Duration& time) : filter_time_constant_(time) {}

void MovingAverage::updateFilter(double value, double derivative, const ros::Time& time) {
    if (!value_buffer_.empty() && time <= value_buffer_.back().stamp) {
        ROS_ERROR("Tried to add point to filter with timestamp older than newest point in filter");
        return;
    }

    value_buffer_.emplace_back(value, time);

    while (value_buffer_.size() > 1) {
        ros::Duration time_since_second_reading =
            value_buffer_.back().stamp - value_buffer_[1].stamp;
        if (time_since_second_reading >= filter_time_constant_) {
            value_buffer_.pop_front();
        } else {
            break;
        }
    }
}

double MovingAverage::getFilteredValue(const ros::Time& time) const {
    if (value_buffer_.empty()) {
        ROS_ERROR("Attempt to get value from empty filter");
        return std::numeric_limits<double>::quiet_NaN();
    }

    if (time - value_buffer_.back().stamp > filter_time_constant_) {
        ROS_WARN("Attempt to get value from filter when newest measurement is older than the filter time");
        return value_buffer_.back().value;
    }

    if (time < value_buffer_.back().stamp) {
        ROS_WARN("Attempt to get value at time before the most recent filter update");
        return value_buffer_.back().value;
    }

    if (value_buffer_.size() == 1) {
        return value_buffer_.back().value;
    }

    std::vector<double> value_vector;
    double output_sum = 0;

    // handle first item
    size_t i;

    ros::Time oldest_sample_stamp = value_buffer_[0].stamp;
    if (oldest_sample_stamp > time - filter_time_constant_) {
        double oldest_sample_weight = (oldest_sample_stamp - time - filter_time_constant_).toSec();
        oldest_sample_weight += (value_buffer_[1].stamp - oldest_sample_stamp).toSec() / 2;
        output_sum += oldest_sample_weight * value_buffer_[0].value;
        i = 0;
    } else {
        ros::Time old_side_cutoff = time - filter_time_constant_;
        for (i = 1; i < value_buffer_.size() - 1; i++) {
            if (value_buffer_[i].stamp > time - filter_time_constant_) {
                // add portion for time between the oldest sample inside the
                //   interval and the halfway point after it
                double dt_next = (value_buffer_[i+1].stamp
                                        - value_buffer_[i].stamp).toSec();
                output_sum += value_buffer_[i].value * dt_next / 2;

                // Now we add the portion for the chunk before the oldest sample
                //   inside the interval.  This takes a sample halfway between
                //   the edge of the window and the oldest sample, and evaluates
                //   there using a linear interpolation between the sample
                //   outside the interval and the one inside
                double dt_prev = (value_buffer_[i].stamp - value_buffer_[i-1].stamp).toSec();
                double weight = (value_buffer_[i].stamp - old_side_cutoff).toSec();
                ros::Time sample_time = old_side_cutoff + ros::Duration(weight / 2);
                double t2 = (value_buffer_[i].stamp - sample_time).toSec() / dt_prev;
                double t1 = (sample_time - value_buffer_[i-1].stamp).toSec() / dt_prev;
                double sample = value_buffer_[i-1].value * t2
                              + value_buffer_[i].value * t1;
                output_sum += sample * weight;

                break;
            }
        }
        ROS_ASSERT_MSG(i < value_buffer_.size() - 1 || value_buffer_.back().stamp >= old_side_cutoff,
                       "The preceding loop should always find a time to start with unless there are only two times");
    }

    // handle middle items
    i++;
    for (; i < value_buffer_.size() - 1; i++) {
        // Each data point counts for half of the interval to its left and half
        //   of the one to its right, so we take the entire interval from the
        //   point on the left to the point on the right and divide by 2.
        ros::Duration duration = value_buffer_[i+1].stamp
            - value_buffer_[i-1].stamp;
        output_sum += value_buffer_[i].value * duration.toSec() / 2;
    }

    // handle last item
    double last_time = value_buffer_.back().stamp.toSec();
    double next_to_last_time = value_buffer_[value_buffer_.size() - 2].stamp.toSec();
    double last_item_weight = time.toSec() - (last_time + next_to_last_time) / 2;
    output_sum += value_buffer_.back().value * last_item_weight;

    return output_sum / filter_time_constant_.toSec();
}

} // namespace iarc7_sensors
