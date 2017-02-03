#include <ros/ros.h>
#include <string.h>

#include "iarc7_sensors/AltimeterFilter.hpp"
#include "lidarlite.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "iarc7_msgs/Float64Stamped.h"

// Attempts to connect to the lidarlite until successful
void connect(LidarLite& lidarLite) {
    bool err;
    ros::Rate rate (100);
    do {
        err = lidarLite.openLidarLite();
        if (err == false) {
            ROS_ERROR("Unable to connect to LidarLite v2, trying to connect");
        }
        ros::spinOnce();
        rate.sleep();
    } while (err == false && ros::ok());
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "altimeter");
    ros::NodeHandle n;

    // Fetch parameters from ROS
    std::string altitude_frame;
    n.param("output_frame", altitude_frame, std::string("lidarlite"));

    double altitude_covariance;
    n.param("altitude_covariance", altitude_covariance, 0.05);

    // Create publishers
    ros::Publisher altitude_pub =
        n.advertise<iarc7_msgs::Float64Stamped>("altimeter_reading", 0);
    ros::Publisher velocity_pub =
        n.advertise<iarc7_msgs::Float64Stamped>("velocity", 0);
    ros::Publisher altitude_pose_pub =
        n.advertise<geometry_msgs::PoseWithCovarianceStamped>("altimeter_pose", 0);

    LidarLite lidarLite;
    iarc7_sensors::AltimeterFilter filter(n, altitude_frame, "level_quad");

    // Connect to the lidarlite
    connect(lidarLite);

    while(ros::ok() && lidarLite.error >= 0){
        ros::spinOnce();

        // Fetch altitude from lidarlite
        int altitude_int;
        bool success = (lidarLite.getDistance(altitude_int) >= 0);
        double altitude = altitude_int / 100.0;

        // Fetch velocity from lidarlite
        int velocity_int;
        success = success && (lidarLite.getVelocity(velocity_int) >= 0);
        double velocity = velocity_int / 100.0;

        if (success) {
            // Create messages to publish
            iarc7_msgs::Float64Stamped altimeter_reading_msg;
            iarc7_msgs::Float64Stamped velocity_msg;
            geometry_msgs::PoseWithCovarianceStamped altimeter_pose_msg;

            // Update filter and lookup transforms
            ros::Time tempTime = ros::Time::now();
            filter.updateFilter(altitude, velocity, tempTime);

            // Publish raw altimeter reading
            altimeter_reading_msg.header.frame_id = altitude_frame;
            altimeter_reading_msg.header.stamp = tempTime;
            altimeter_reading_msg.data = altitude;
            altitude_pub.publish(altimeter_reading_msg);

            // Publish raw velocity reading
            velocity_msg.header.frame_id = altitude_frame;
            velocity_msg.header.stamp = tempTime;
            velocity_msg.data = velocity;
            velocity_pub.publish(velocity_msg);

            // Publish pose estimate
            altimeter_pose_msg.header.frame_id = "map";
            altimeter_pose_msg.header.stamp = tempTime;
            altimeter_pose_msg.pose.pose.position.z =
                filter.getFilteredAltitude(tempTime);

            // This is a 6x6 matrix and z height is variable 2,
            // so the z height covariance is at location (2,2)
            altimeter_pose_msg.pose.covariance[2*6 + 2] = altitude_covariance;
            altitude_pose_pub.publish(altimeter_pose_msg);

        } else {
            ROS_ERROR("Lidar-Lite communication failed");
            connect(lidarLite);
            ROS_ERROR("Successfully reestablished connection with LidarLite v2");
        }

    }

    return 0;
}
