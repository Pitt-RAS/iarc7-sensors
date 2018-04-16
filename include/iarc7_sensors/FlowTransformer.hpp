
#ifndef IARC7_SENSORS_FLOW_TRANSFORMER_HPP_
#define IARC7_SENSORS_FLOW_TRANSFORMER_HPP_


#include <ros/ros.h>
#include <ros_utils/SafeTransformWrapper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <iarc7_msgs/FlowVector.h>

namespace iarc7_sensors {

struct FlowTransformerSettings {
    double fov; // 0.73 rad
    double min_estimation_altitude; // Future reference - this is 80 mm - some transform tree value
    double vertical_threshold; //
    double variance;
    double variance_scale;
    double tf_timeout;
    int image_width;
    bool debug_print;
};

// variables - current / last orientation, target_size, current_altitude

class FlowTransformer {

public:

    //////////////////
    // CONSTRUCTORS //
    //////////////////

    FlowTransformer(
    const FlowTransformerSettings& flow_transformer_settings,
    ros::NodeHandle nh);


    void updateVelocity(iarc7_msgs::FlowVector);

private:



    geometry_msgs::TwistWithCovarianceStamped
    estimateVelocityFromFlowVector(const int deltaX, const int deltaY, const ros::Time& time);


    bool updateFilteredPosition(const ros::Time& time, const ros::Duration& timeout);


    void getYPR(const tf2::Quaternion& orientation,
                                  double& y,
                                  double& p,
                                  double& r);

    bool canEstimateFlow();




    ////////////////////////
    // INSTANCE VARIABLES //
    ////////////////////////


    double current_altitude_;
    tf2::Quaternion current_orientation_;
    tf2::Quaternion last_orientation_;
    geometry_msgs::TransformStamped current_camera_to_level_quad_tf_;
    geometry_msgs::TransformStamped last_camera_to_level_quad_tf_;
    const ros_utils::SafeTransformWrapper transform_wrapper_;

    iarc7_sensors::FlowTransformerSettings flow_transformer_settings;

    /// Timestamp from last message received
    ros::Time last_message_time_;
    
    // Publisher/s
    const ros::Publisher twist_pub_;

    const ros::Publisher debug_correction_pub_;
    const ros::Publisher debug_raw_pub_;
    const ros::Publisher debug_unrotated_vel_pub_;


    // Filter that gets applied to x/y velocity calculations to prevent random zero
    // crossings that should not exist
    //
    // velocity_filtered[0] has the x measurements
    // [1] has the y measurements 
    //const float filter_coefs_[4] = {0.0677, 0.4323, 0.4323, 0.0677};
    const float filter_coefs_[5] = {0.0338, 0.2401, 0.4521, 0.2401, 0.0338};
    const int filter_order_ = 4;
    float velocity_filtered_[2][5];

    };

}
#endif // include guard
