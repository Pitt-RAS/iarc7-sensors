#include "iarc7_sensors/FlowTransformer.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iarc7_msgs/FlowVector.h>
#include <math.h>
#include <ros/ros.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wignored-attributes"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#include <Eigen/Geometry>
#pragma GCC diagnostic pop

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iarc7_msgs/OrientationAnglesStamped.h>
#include <ros_utils/SafeTransformWrapper.hpp>


void getFlowTransformerSettings(const ros::NodeHandle& private_nh,
                          iarc7_sensors::FlowTransformerSettings& settings)
{
    ROS_ASSERT(private_nh.getParam(
        "fov",
        settings.fov));

    ROS_ASSERT(private_nh.getParam(
        "min_estimation_altitude",
        settings.min_estimation_altitude));

    ROS_ASSERT(private_nh.getParam(
        "vertical_threshold",
        settings.vertical_threshold));

    ROS_ASSERT(private_nh.getParam(
        "tf_timeout",
        settings.tf_timeout));

    ROS_ASSERT(private_nh.getParam(
        "image_width",
        settings.image_width));

    ROS_ASSERT(private_nh.getParam(
        "variance",
        settings.variance));

    ROS_ASSERT(private_nh.getParam(
        "variance_scale",
        settings.variance_scale));

    ROS_ASSERT(private_nh.getParam(
        "debug_print",
        settings.debug_print));

}


int main(int argc, char* argv[]){

    ros::init(argc, argv, "optical_flow_estimator");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    iarc7_sensors::FlowTransformerSettings flow_transformer_settings;
    getFlowTransformerSettings(private_nh, flow_transformer_settings);

    iarc7_sensors::FlowTransformer flow_transformer(flow_transformer_settings, private_nh);

    // process will go - subsriber gets message with flow vectors
    // calls updateVelocity to get current orientation and altitude
    // updateVelocity performs basic checks, gets current position.velocity 
    // from other topics
    // Finally, call estimateVelocity, which actually does math on rotation/flow vectors
    // Gets twist msg, checks for infinity, and then publishes

    boost::function<void (const iarc7_msgs::FlowVector&)> callback = 
    [&] (const iarc7_msgs::FlowVector& flow_vector){
        flow_transformer.updateVelocity(flow_vector);
    };

    ros::Subscriber Flow_vector_sub = nh.subscribe<iarc7_msgs::FlowVector>("flow_vector",
                                 0,
                                 callback);

    ros::spin();

}


namespace iarc7_sensors{

FlowTransformer::FlowTransformer(
    const FlowTransformerSettings& flow_transformer_settings,
                ros::NodeHandle nh)
                :current_altitude_(0.0),
                current_orientation_(),
                last_orientation_(),
                last_camera_to_level_quad_tf_(),
                transform_wrapper_(),
                flow_transformer_settings(flow_transformer_settings),
                twist_pub_(
                    nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(
                    "twist", 10)),
                debug_correction_pub_(
                    nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(
                    "twist_correction", 10)),
                debug_raw_pub_(
                    nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(
                    "twist_raw", 10)),
                debug_unrotated_vel_pub_(
                    nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(
                    "twist_unrotated", 10))
{
    return;
}


geometry_msgs::TwistWithCovarianceStamped 
FlowTransformer::estimateVelocityFromFlowVector(const int deltaX, const int deltaY, const ros::Time& time)
{

    geometry_msgs::TwistWithCovarianceStamped twist;

    ROS_ERROR("At beginning of estimate Velocity");
    // Get the pitch and roll of the camera in euler angles
    // NOTE: CAMERA FRAME CONVENTIONS ARE DIFFERENT, SEE REP103
    // http://www.ros.org/reps/rep-0103.html
    double yaw, pitch, roll;
    getYPR(current_orientation_, yaw, pitch, roll);

    double last_yaw, last_pitch, last_roll;
    getYPR(last_orientation_, last_yaw, last_pitch, last_roll);

    // Calculate time between last and current frame
    double dt = (time - last_message_time_).toSec();
    
    ROS_ERROR("time is %f\n", time.toSec());
    ROS_ERROR("last time is %f\n", last_message_time_.toSec());

    ROS_ERROR("dt is %lf\n", dt);    

    // Distance from the camera to the ground plane, along the camera's +z axis
    //
    // Calculation based on a right triangle with one vertex at the camera,
    // one vertex on the ground directly below the camera, and one vertex at
    // the intersection of the camera's forward vector with the ground.  This
    // calculates the hypotenuse if the vertical leg has length
    // current_altitude_ and the horizontal leg has length
    // ((current_altitude_*tan(pitch))^2 + (current_altitude_*tan(roll))^2)^0.5
    ROS_ERROR_STREAM("Current altitude before use: " << current_altitude_);
    double distance_to_plane = current_altitude_
                             * std::sqrt(1.0
                            + std::pow(std::tan(pitch), 2.0)
                            + std::pow(std::tan(roll), 2.0));

    ROS_ERROR("distance_to_plane is %lf\n", distance_to_plane);
    
    
    // Multiplier that converts measurements in pixels to measurements in
    // meters in the plane parallel to the camera's xy plane and going through
    // the point Pg, where Pg is intersection between the camera's +z axis and
    // the ground plane
    //
    // Focal length gives distance from camera to image plane in pixels, so
    // dividing distance to plane by this number gives the multiplier we want
    // dtp * tan 42 /15 px

    double current_meters_per_px = distance_to_plane * std::tan(flow_transformer_settings.fov/2)
                                    / (flow_transformer_settings.image_width/2.0);

    ROS_ERROR_STREAM("current meters per pix: " << current_meters_per_px);

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // These velocities are flipped, but it really should be implemented properly in the 
    // transform tree. Definitely fix this later.
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    float estimatedXVel = current_meters_per_px * deltaX / std::cos(pitch)/ dt;
    float estimatedYVel = current_meters_per_px * deltaY / -std::cos(roll) / dt;
    ROS_ERROR("estimatedX, %f dX, %d, estimatedY, %f, dY, %d\n", estimatedXVel, deltaX, estimatedYVel, deltaY);

    // Move the samples taken forward in time
    for(int i = 0; i < filter_order_; i++)
    {
        velocity_filtered_[0][i+1] = velocity_filtered_[0][i];
        velocity_filtered_[1][i+1] = velocity_filtered_[0][i];
    }

    velocity_filtered_[0][0] = estimatedXVel;
    velocity_filtered_[1][0] = estimatedYVel;


    float filteredXVel = 0;
    float filteredYVel = 0;


    for(int i = 0; i < filter_order_ + 1; i++)
    {
        filteredXVel += velocity_filtered_[0][i] * filter_coefs_[i];
        filteredYVel += velocity_filtered_[1][i] * filter_coefs_[i];
    }

    //filteredXVel = estimatedXVel;
    //filteredYVel = estimatedYVel;

    double dp;
    double dr;

    // These two if statements make sure that dp and dr are the shortest change
    // in angle that would produce the new observed orientation
    //
    // i.e. a change from 0.1rad to (2pi-0.1)rad should result in a delta of
    // -0.2rad, not (2pi-0.2)rad
    if (last_pitch > M_PI/2 && pitch < -M_PI/2) {
        dp = (pitch + 2*M_PI - last_pitch);
    } else if (last_pitch < -M_PI/2 && pitch > M_PI/2) {
        dp = (pitch - last_pitch - 2*M_PI);
    } else {
        dp = (pitch - last_pitch);
    }

    if (last_roll > M_PI/2 && roll < -M_PI/2) {
        dr = (roll + 2*M_PI - last_roll);
    } else if (last_roll < -M_PI/2 && roll > M_PI/2) {
        dr = (roll - last_roll - 2*M_PI);
    } else {
        dr = (roll - last_roll);
    }
    ROS_ERROR_STREAM("dp and dr are " << dp << " "  << dr);
    

    double dpitch_dt = dp / dt;
    double droll_dt = dr / dt;
    ROS_ERROR_STREAM("dp/dt is "<< dpitch_dt << "and dr/dt is " << droll_dt);


    //float correctedXVel = distance_to_plane
    //                 * -dpitch_dt
    //                 / std::cos(pitch);
    //float correctedYVel = distance_to_plane
    //                 * -droll_dt
    //                 / -std::cos(roll);
    //ROS_ERROR_STREAM("correctedXVel is" << correctedXVel << "and correctedYVel is" << correctedYVel);

    
    // Actual velocity in level camera frame (i.e. camera frame if our pitch
    // and roll were zero)
    Eigen::Vector3d corrected_vel(
        filteredXVel, // - correctedXVel,
        filteredYVel, // - correctedYVel,
        0.0);
    // Calculate covariance
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    covariance(0, 0) = std::pow(flow_transformer_settings.variance
                       * current_meters_per_px / dt, 2.0);
    covariance(1, 1) = std::pow(flow_transformer_settings.variance
                       * current_meters_per_px / dt, 2.0);

    // Rotation matrix from level camera frame to level_quad
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ());

    // Get velocity and covariance in level_quad frame
    Eigen::Vector3d level_quad_vel = rotation_matrix * corrected_vel;
    Eigen::Matrix3d level_quad_covariance = rotation_matrix
                                          * covariance
                                          * rotation_matrix.inverse();

    // Correct for movement of camera frame relative to level_quad
    //
    // When the drone rotates the camera has some velocity relative to the
    // ground even if the center of the drone doesn't move relative to the
    // ground, this cancels that effect
    /*
    geometry_msgs::PointStamped curr_pos, last_pos;
    tf2::doTransform(curr_pos, curr_pos, current_camera_to_level_quad_tf_);
    tf2::doTransform(last_pos, last_pos, last_camera_to_level_quad_tf_);
    double camera_relative_vel_x = (curr_pos.point.x - last_pos.point.x) / dt;
    double camera_relative_vel_y = (curr_pos.point.y - last_pos.point.y) / dt;
    */

    Eigen::Vector3d corrected_level_quad_vel = level_quad_vel;
    //corrected_level_quad_vel.x() -= camera_relative_vel_x;
    //corrected_level_quad_vel.y() -= camera_relative_vel_y;

    // Fill out the twist
    twist.header.stamp = time;
    twist.header.frame_id = "level_quad";
    twist.twist.twist.linear.x = corrected_level_quad_vel.x();
    twist.twist.twist.linear.y = corrected_level_quad_vel.y();
    twist.twist.twist.linear.z = 0.0;

    twist.twist.covariance[0] = level_quad_covariance(0, 0);
    twist.twist.covariance[1] = level_quad_covariance(0, 1);
    twist.twist.covariance[6] = level_quad_covariance(1, 0);
    twist.twist.covariance[7] = level_quad_covariance(1, 1);

    // Publish intermediate twists for debugging
    if (flow_transformer_settings.debug_print) {

        geometry_msgs::TwistWithCovarianceStamped twist_raw = twist;
        twist_raw.twist.twist.linear.x = estimatedXVel;
        twist_raw.twist.twist.linear.y = estimatedYVel;
        debug_raw_pub_.publish(twist_raw);

        // Corrected X/Y vel takes the velocity from pitch/roll changes into account
        //geometry_msgs::TwistWithCovarianceStamped twist_correction = twist;
        //twist_correction.twist.twist.linear.x = correctedXVel;
        //twist_correction.twist.twist.linear.y = correctedYVel;
        //debug_correction_pub_.publish(twist_correction);

        geometry_msgs::TwistWithCovarianceStamped twist_unrotated = twist;
        twist_unrotated.twist.twist.linear.x = corrected_vel.x();
        twist_unrotated.twist.twist.linear.y = corrected_vel.y();
        debug_unrotated_vel_pub_.publish(twist_unrotated);

    }
    return twist;

}


void FlowTransformer::getYPR(const tf2::Quaternion& orientation,
                                  double& y,
                                  double& p,
                                  double& r)
{
    tf2::Matrix3x3 matrix;
    matrix.setRotation(orientation);
    matrix.getEulerYPR(y, p, r);
}



void FlowTransformer::updateVelocity(iarc7_msgs::FlowVector flow_vector)
{


    // make sure our current position is up to date
    if (!updateFilteredPosition(
                flow_vector.header.stamp,
                ros::Duration(flow_transformer_settings.tf_timeout))) {
        ROS_ERROR("Unable to update position for optical flow");

        // Add something to tell this not to publish
        // But is that necessary if we return here?

        //have_valid_last_image_ = false;
        return;
    }

    // Make sure we're in an allowed position to calculate optical flow
    if (!canEstimateFlow()) {
        // Add something to tell this not to publish

        //have_valid_last_image_ = false;
        return;
    }


    const geometry_msgs::TwistWithCovarianceStamped velocity
        = estimateVelocityFromFlowVector(flow_vector.deltaX, 
        flow_vector.deltaY,
        flow_vector.header.stamp);

    ROS_INFO("Before checking whether we have some infinite numbers");

    last_message_time_ = flow_vector.header.stamp;

    if (!std::isfinite(velocity.twist.twist.linear.x)
     || !std::isfinite(velocity.twist.twist.linear.y)
     || !std::isfinite(velocity.twist.covariance[0])    // variance of x
     || !std::isfinite(velocity.twist.covariance[1])    // covariance of x & y
     || !std::isfinite(velocity.twist.covariance[6])    // covariance of x & y
     || !std::isfinite(velocity.twist.covariance[7])) { // variance of y
        ROS_ERROR_STREAM("Invalid measurement in OpticalFlowEstimator: "
                      << velocity);
    } else {
        // Publish velocity estimate
        twist_pub_.publish(velocity);

    }
    last_message_time_ = flow_vector.header.stamp;
    last_orientation_ = current_orientation_;
    last_camera_to_level_quad_tf_ = current_camera_to_level_quad_tf_;
}



bool FlowTransformer::canEstimateFlow()
{
    if (current_altitude_ < flow_transformer_settings.min_estimation_altitude) {
        ROS_WARN_THROTTLE(2.0,
                          "Optical flow: height (%f) is below min processing height (%f)",
                          current_altitude_,
                          flow_transformer_settings.min_estimation_altitude);
        return false;
    }
    else{

        return true;
    }

}

bool FlowTransformer::updateFilteredPosition(const ros::Time& time,
                                             const ros::Duration& timeout)
{
    geometry_msgs::TransformStamped filtered_position_transform_stamped;
    geometry_msgs::TransformStamped camera_to_level_quad_tf_stamped;


    // Modify transforms
    bool success = transform_wrapper_.getTransformAtTime(
            filtered_position_transform_stamped,
            "map",
            "pmw3901_optical_flow_optical",
            time,
            timeout);

    if (!success) {
        return false;
    }

    // Modify transforms
    success = transform_wrapper_.getTransformAtTime(
            camera_to_level_quad_tf_stamped,
            "level_quad",
            "pmw3901_optical_flow_optical",
            time,
            timeout);

    if (!success) {
        return false;
    }

    geometry_msgs::PointStamped camera_position;
    tf2::doTransform(camera_position,
                     camera_position,
                     filtered_position_transform_stamped);
    current_altitude_ = camera_position.point.z;

    tf2::convert(filtered_position_transform_stamped.transform.rotation,
                 current_orientation_);

    current_camera_to_level_quad_tf_ = camera_to_level_quad_tf_stamped;
    return true;
}
}

