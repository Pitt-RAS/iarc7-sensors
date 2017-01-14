#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include "lidarlite.hpp"
#include "ros/ros.h"
#include "iarc7_msgs/Float64Stamped.h"
#include <limits>
#include "iarc7_sensors/AltimeterFilter.hpp"

#include <sstream>


int main(int argc, char **argv) {

    ros::init(argc, argv, "altimeter");
    ros::NodeHandle n;

    std::string altitude_frame;
    n.param("output_frame", altitude_frame, std::string("altimeter_frame"));

    ros::Publisher altitude_pub = n.advertise<iarc7_msgs::Float64Stamped>("altitude", 0);
    ros::Publisher velocity_pub = n.advertise<iarc7_msgs::Float64Stamped>("velocity", 0);

    LidarLite lidarLite;
    iarc7_sensors::AltimeterFilter filter(n, "altimeter_frame", "level_quad");
    
    bool err;
    do {
        ros::spinOnce();
        err = lidarLite.openLidarLite();
        if (err == false) {
            ROS_ERROR("Unable to connect to LidarLite v2, trying to connect");
        }
    } while (err == false);

    
    while(ros::ok() && lidarLite.error >= 0){
        ros::spinOnce();

        int altitude_int;
        bool success = (lidarLite.getDistance(altitude_int) >= 0);
        double altitude = altitude_int / 100.0;

        int velocity_int;
        success = success && (lidarLite.getVelocity(velocity_int) >= 0);
        double velocity = velocity_int / 100.0;

        // altitude_msg.data = altitude;
        
        if (success) {
            iarc7_msgs::Float64Stamped altitude_msg;
            iarc7_msgs::Float64Stamped velocity_msg;
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
        }
        else {
            do {
                ros::spinOnce();
                ROS_ERROR("Lost connection with LidarLite v2, trying to connect");
                lidarLite.closeLidarLite();
                err = lidarLite.openLidarLite();
            } while (err == false);
            ROS_ERROR("Successfully established connection with LidarLite v2"); 
        }
        
    }
}
