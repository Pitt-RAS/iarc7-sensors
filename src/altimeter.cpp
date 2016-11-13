#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include "lidarlite.hpp"
#include "ros/ros.h"
#include "iarc7_msgs/Float64Stamped.h"
#include <limits>

#include <sstream>


int main(int argc, char **argv) {

    ros::init(argc, argv, "altimeter");
    ros::NodeHandle n;
    ros::Publisher altitude_pub = n.advertise<iarc7_msgs::Float64Stamped>("altitude", 0);
    ros::Publisher velocity_pub = n.advertise<iarc7_msgs::Float64Stamped>("velocity", 0);

    LidarLite lidarLite;
    int err = lidarLite.openLidarLite();
    if (err >= 0) {
        while(ros::ok() && lidarLite.error >= 0){
            iarc7_msgs::Float64Stamped altitude_msg;
            altitude_msg.header.stamp = ros::Time::now();
            double altitude = (lidarLite.getDistance()) / 100.0;
            altitude_msg.data = altitude;
            if (altitude >= 0) {
                iarc7_msgs::Float64Stamped velocity_msg;
                velocity_msg.header.stamp = ros::Time::now();
                double velocity = (lidarLite.getVelocity()) / 100.0;
                velocity_msg.data = velocity;
                velocity_pub.publish(velocity_msg);	
            }
            else {
                altitude = std::numeric_limits<double>::quiet_NaN();
            }
            altitude_pub.publish(altitude_msg);
        }
    }
}
