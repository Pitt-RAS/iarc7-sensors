#include <cmath>
#include <ros/ros.h>
#include <string.h>

#include "iarc7_sensors/AltimeterFilter.hpp"
#include "lidarlite.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iarc7_msgs/Float64Stamped.h>
#include <sensor_msgs/Range.h>
#include "iarc7_safety/SafetyClient.hpp"

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

    ROS_INFO("altimeter: Attempting to form safety bond");
    Iarc7Safety::SafetyClient safety_client(n, "low_level_motion");
    ROS_ASSERT_MSG(safety_client.formBond(),
                   "altimeter: Could not form bond with safety client");

    // Fetch parameters from ROS
    std::string altitude_frame;
    private_nh.param("output_frame", altitude_frame, std::string(""));

    double altitude_variance;
    private_nh.param("altitude_variance", altitude_variance, 0.0);

    // Create publishers
    ros::Publisher altitude_pub =
        n.advertise<sensor_msgs::Range>("altimeter_reading", 0);
    ros::Publisher velocity_pub =
        n.advertise<iarc7_msgs::Float64Stamped>("velocity", 0);

    LidarLite lidarLite;
    iarc7_sensors::AltimeterFilter filter(
            n,
            altitude_frame,
            [=](double altitude) -> double {
                return altitude_variance
                     * (1 + 60*std::exp(-250*std::pow(altitude, 4)));
            },
            "level_quad");

    // Connect to the lidarlite
    connect(lidarLite);

    while(ros::ok() && lidarLite.error >= 0){

        ROS_ASSERT_MSG(!safety_client.isFatalActive(),
                       "altimeter: fatal event from safety");
        ros::spinOnce();

        // Fetch altitude from lidarlite
        int altitude_int;
        bool success = (lidarLite.getDistance(altitude_int) >= 0);
        double altitude = altitude_int / 100.0;

        // Fetch velocity from lidarlite
        int velocity_int;
        success = success && (lidarLite.getVelocity(velocity_int) >= 0);
        double velocity = velocity_int / 100.0;

        if(safety_client.isSafetyActive()){
            ROS_ERROR("altimeter has died");
            return 1;
        }else if (success) {
            // Create messages to publish
            sensor_msgs::Range altimeter_reading_msg;
            iarc7_msgs::Float64Stamped velocity_msg;

            // Update filter and lookup transforms
            ros::Time tempTime = ros::Time::now();

            // Publish raw altimeter reading
            altimeter_reading_msg.header.frame_id = altitude_frame;
            altimeter_reading_msg.header.stamp = tempTime;
            altimeter_reading_msg.radiation_type = sensor_msgs::Range::INFRARED;
            altimeter_reading_msg.min_range = 0;
            altimeter_reading_msg.max_range = 100;
            altimeter_reading_msg.range = altitude;
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
