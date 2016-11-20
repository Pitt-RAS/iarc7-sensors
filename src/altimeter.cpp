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
    ros::Publisher altitude_pub = n.advertise<iarc7_msgs::Float64Stamped>("altitude", 0);
    ros::Publisher velocity_pub = n.advertise<iarc7_msgs::Float64Stamped>("velocity", 0);

    LidarLite lidarLite;
    iarc7_sensors::AltimeterFilter filter(n, "altimeter_frame", "level_quad");
    
    bool err;
    do {
        err = lidarLite.openLidarLite();
    } while (err == false);

    
    while(ros::ok() && lidarLite.error >= 0){
        double altitude = (lidarLite.getDistance()) / 100.0;
        double velocity = (lidarLite.getVelocity()) / 100.0;
        // altitude_msg.data = altitude;
        
        if (altitude >= 0 && velocity >= 0) {
            iarc7_msgs::Float64Stamped altitude_msg;
            iarc7_msgs::Float64Stamped velocity_msg;
            ros::Time tempTime = ros::Time::now();
            filter.updateFilter(altitude, velocity, tempTime);
            altitude_msg.header.stamp = tempTime;
            velocity_msg.header.stamp = tempTime;
            altitude = filter.getFilteredAltitude(tempTime);
            altitude_msg.data = altitude; 
            velocity_msg.data = velocity;
            altitude_pub.publish(altitude_msg);
            velocity_pub.publish(velocity_msg);	
        }
        else {
            do {
                lidarLite.closeLidarLite();
                err = lidarLite.openLidarLite();
            } while (err == false);
        }
        
    }
}
