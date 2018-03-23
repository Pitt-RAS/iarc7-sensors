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


    estimatedXmeters = current_meters_per_px * deltaX;
    estimatedYmeters = current_meters_per_px * deltaY;
    




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






/*

bool FlowTransformer::updateFilteredPosition(const ros::Time& time,
                                                  const ros::Duration& timeout)
{
    geometry_msgs::TransformStamped filtered_position_transform_stamped;
    geometry_msgs::TransformStamped camera_to_level_quad_tf_stamped;

    bool success = transform_wrapper_.getTransformAtTime(
            filtered_position_transform_stamped,
            "map",
            "bottom_camera_rgb_optical_frame",
            time,
            timeout);

    if (!success) {
        return false;
    }

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

*/