#include <iarc7_sensors/OpticalFlowEstimator.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iarc7_msgs/FlowVector.h>
#include <ros/ros.h>
#include <Eigen/Geometry>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iarc7_msgs/OrientationAnglesStamped.h>

// Thank you, Aaron

int main(int argc, char* argv[]){

    ros::init(argc, argv, "Optical_flow_estimator");
    ros::NodeHandle nh;

    // process will go - subsriber gets message with flow vectors
    // calls updateFilteredPosition to get current orientation and altitude
    // Subscriber then calls estimateVelocityFromFlowVector 
    // Gets twist msg, checks for infinity, and then publishes

    boost::function<void (const iarc7_msgs::FlowVector&)> callback = 
    [&] (const iarc7_msgs::FlowVector& flow_vector){
        updateVelocity(flow_vector);
    };


    // & just means "look at all the objects already declared and pull the ones you need"
    ros::Subscriber Flow_vector_sub = nh.subscribe<iarc7_msgs::FlowVector>("flow_vector",
                                 0,
                                 callback);

}



geometry_msgs::TwistWithCovarianceStamped
FlowTransformer::estimateVelocityFromFlowVector(const int deltaX, const int deltaY, const ros::Time& time){

   // Get the pitch and roll of the camera in euler angles
    // NOTE: CAMERA FRAME CONVENTIONS ARE DIFFERENT, SEE REP103
    // http://www.ros.org/reps/rep-0103.html
    double yaw, pitch, roll;
    getYPR(current_orientation_, yaw, pitch, roll);

    double last_yaw, last_pitch, last_roll;
    getYPR(last_orientation_, last_yaw, last_pitch, last_roll);

    // Calculate time between last and current frame
    double dt = (time - last_message_time_).toSec();


    // Distance from the camera to the ground plane, along the camera's +z axis
    //
    // Calculation based on a right triangle with one vertex at the camera,
    // one vertex on the ground directly below the camera, and one vertex at
    // the intersection of the camera's forward vector with the ground.  This
    // calculates the hypotenuse if the vertical leg has length
    // current_altitude_ and the horizontal leg has length
    // ((current_altitude_*tan(pitch))^2 + (current_altitude_*tan(roll))^2)^0.5
    double distance_to_plane = current_altitude_
                             * std::sqrt(1.0
                            + std::pow(std::tan(pitch), 2.0)
                            + std::pow(std::tan(roll), 2.0));

    // Multiplier that converts measurements in pixels to measurements in
    // meters in the plane parallel to the camera's xy plane and going through
    // the point Pg, where Pg is intersection between the camera's +z axis and
    // the ground plane
    //
    // Focal length gives distance from camera to image plane in pixels, so
    // dividing distance to plane by this number gives the multiplier we want
    // dtp * tan 42 /15 px
    double current_meters_per_px = distance_to_plane * std::tan(flow_transformer_settings_.fov)
                                    / flow_transformer_settings_.pix_width;



    estimatedXVel = current_meters_per_px * deltaX / dt;
    estimatedYVel = current_meters_per_px * deltaY / dt;
    

    // For now, copy pasting all of Aaron's covariance stuff so that the rest of the 
    // flight stack still works

    // Calculate covariance
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    covariance(0, 0) = std::pow(flow_transformer_settings_.variance_scale
                              * dpitch_dt, 2.0)
                      + flow_transformer_settings_.variance;
    covariance(1, 1) = std::pow(flow_transformer_settings_.variance_scale
                              * droll_dt, 2.0)
                      + flow_transformer_settings_.variance;

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
    geometry_msgs::PointStamped curr_pos, last_pos;
    tf2::doTransform(curr_pos, curr_pos, current_camera_to_level_quad_tf_);
    tf2::doTransform(last_pos, last_pos, last_camera_to_level_quad_tf_);
    double camera_relative_vel_x = (curr_pos.point.x - last_pos.point.x) / dt;
    double camera_relative_vel_y = (curr_pos.point.y - last_pos.point.y) / dt;

    Eigen::Vector3d corrected_level_quad_vel = level_quad_vel;
    corrected_level_quad_vel.x() -= camera_relative_vel_x;
    corrected_level_quad_vel.y() -= camera_relative_vel_y;

    // Fill out the twist
    geometry_msgs::TwistWithCovarianceStamped twist;
    twist.header.stamp = time;
    twist.header.frame_id = "level_quad";
    twist.twist.twist.linear.x = estimatedYVel;
    twist.twist.twist.linear.y = estimatedXVel;
    twist.twist.twist.linear.z = 0.0;

    twist.twist.covariance[0] = level_quad_covariance(0, 0);
    twist.twist.covariance[1] = level_quad_covariance(0, 1);
    twist.twist.covariance[6] = level_quad_covariance(1, 0);
    twist.twist.covariance[7] = level_quad_covariance(1, 1);

return twist;

}

void FlowTransformer::getFocalLength()

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
                ros::Duration(flow_transformer_settings_.tf_timeout))) {
        ROS_ERROR("Unable to update position for optical flow");

        // Add something to tell this not to publish

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

}



bool FlowTransformer::canEstimateFlow()
{
    if (current_altitude_ < flow_transformer_settings_.min_estimation_altitude) {
        ROS_WARN_THROTTLE(2.0,
                          "Optical flow: height (%f) is below min processing height (%f)",
                          current_altitude_,
                          flow_transformer_settings_.min_estimation_altitude);
        return false;

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
            "bottom_camera_rgb_optical_frame",
            time,
            timeout);

    if (!success) {
        return false;
    }

    // Modify transforms
    success = transform_wrapper_.getTransformAtTime(
            camera_to_level_quad_tf_stamped,
            "level_quad",
            "bottom_camera_rgb_optical_frame",
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
