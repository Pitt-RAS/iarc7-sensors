#include <ros/ros.h>
#include <string.h>

#include "iarc7_sensors/AltimeterFilter.hpp"
#include "lidarlite.hpp"

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
    n.param("output_frame", altitude_frame, std::string("altimeter_frame"));

    ros::Publisher altitude_pub = n.advertise<iarc7_msgs::Float64Stamped>("altitude", 0);
    ros::Publisher velocity_pub = n.advertise<iarc7_msgs::Float64Stamped>("velocity", 0);

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
            iarc7_msgs::Float64Stamped altitude_msg;
            iarc7_msgs::Float64Stamped velocity_msg;

            // Update filter and lookup transforms
            ros::Time tempTime = ros::Time::now();
//            filter.updateFilter(altitude, velocity, tempTime);
            altitude_msg.header.frame_id = altitude_frame;
            altitude_msg.header.stamp = tempTime;
            velocity_msg.header.frame_id = altitude_frame;
            velocity_msg.header.stamp = tempTime;
//            altitude = filter.getFilteredAltitude(tempTime);
            altitude_msg.data = altitude; 
            velocity_msg.data = velocity;
            altitude_pub.publish(altitude_msg);
            velocity_pub.publish(velocity_msg);	
        } else {
            ROS_ERROR("Lidar-Lite communication failed");
            connect(lidarLite);
            ROS_ERROR("Successfully reestablished connection with LidarLite v2");
        }

    }

    return 0;
}
