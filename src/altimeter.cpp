#include <cmath>
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

    err = lidarLite.openLidarLite();
    if (err == false) {
        ROS_ERROR("Unable to connect to LidarLite v2, attempting connection again");
        do {
            ros::spinOnce();
            rate.sleep();

            err = lidarLite.openLidarLite();
            if (err == false) {
                ROS_DEBUG("Connection still not established with the lidarlite");
            }
        } while (err == false && ros::ok());
    }
}

void reconnect(LidarLite& lidarLite)
{
    ROS_ERROR("Reconnection to lidarlite requested");
    connect(lidarLite);
    ROS_ERROR("Successfully reestablished connection with LidarLite v2");
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "altimeter");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    // Fetch parameters from ROS
    std::string altitude_frame;
    private_nh.param("output_frame", altitude_frame, std::string("lidarlite"));

    double altitude_covariance;
    private_nh.param("altitude_covariance", altitude_covariance, 0.05);

    // Create publishers
    ros::Publisher altitude_pub =
        n.advertise<iarc7_msgs::Float64Stamped>("altimeter_reading", 0);
    ros::Publisher velocity_pub =
        n.advertise<iarc7_msgs::Float64Stamped>("velocity", 0);

    LidarLite lidarLite;
    iarc7_sensors::AltimeterFilter filter(n,
                                          altitude_frame,
                                          altitude_covariance,
                                          "level_quad");

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

            // Update filter and lookup transforms
            ros::Time tempTime = ros::Time::now();

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
        } else {
            ROS_ERROR("Lidar-Lite communication failed");
            reconnect(lidarLite);
        }
    }

    return 0;
}
