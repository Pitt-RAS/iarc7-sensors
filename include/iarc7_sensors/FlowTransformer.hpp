
#ifndef IARC7_SENSORS_FLOW_TRANSFORMER_HPP_
#define IARC7_SENSORS_FLOW_TRANSFORMER_HPP_


#include <ros/ros.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros_utils/SafeTransformWrapper.hpp>

namespace iarc7_vision {

struct FlowTransformerEstimatorSettings {
    double fov; // 0.73 rad
    double min_estimation_altitude; // Future reference - this is 80 mm - some transform tree value
    double target_size;
    double camera_vertical_threshold; //
    //int win_size;
    double variance;
    double variance_scale;
    double tf_timeout;
};

// variables - current / last orientation, target_size, current_altitude

class FlowTransformer {

public:



private:



    geometry_msgs::TwistWithCovarianceStamped
    FlowTransformer::estimateVelocityFromFlowVector(const int deltaX, const int deltaY, const ros::Time& time);


    bool FlowTransformer::updateFilteredPosition(const ros::Time& time, const ros::Duration& timeout);


    void OpticalFlowEstimator::getYPR(const tf2::Quaternion& orientation,
                                  double& y,
                                  double& p,
                                  double& r);

    ////////////////////////
    // INSTANCE VARIABLES //
    ////////////////////////


    double current_altitude_;
    tf2::Quaternion current_orientation_;
    tf2::Quaternion last_orientation_;
    geometry_msgs::TransformStamped current_camera_to_level_quad_tf_;
    geometry_msgs::TransformStamped last_camera_to_level_quad_tf_;

    /// Timestamp from last message received
    ros::Time last_message_time_;


}


#endif // include guard