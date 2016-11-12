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


int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "altimeter");
    ros::NodeHandle n;
    ros::Publisher altitude_pub = n.advertise<iarc7_msgs::Float64Stamped>("altitude", 0);
    ros::Publisher velocity_pub = n.advertise<iarc7_msgs::Float64Stamped>("velocity", 0);

    LidarLite *lidarLite = new LidarLite() ;
    int err = lidarLite->openLidarLite();
    if (err < 0){
        //printf("Error: %d", lidarLite->error);
    } else {

        int hardwareVersion = lidarLite->getHardwareVersion() ;
        int softwareVersion = lidarLite->getSoftwareVersion() ;

        // 27 is the ESC key
        int count = 0;
  
        while(ros::ok() && lidarLite->error >= 0 && getkey() != 27){
            iarc7_msgs::Float64Stamped altitude_msg;
            altitude_msg.header.stamp = ros::Time::now();
            double altitude = (lidarLite->getDistance()) / 100.0;
            altitude_msg.data = altitude;
            if (altitude >= 0) {
                iarc7_msgs::Float64Stamped velocity_msg;
                velocity_msg.header.stamp = ros::Time::now();
                double velocity = (lidarLite->getVelocity()) / 100.0;
                velocity_msg.data = velocity;
                velocity_pub.publish(velocity_msg);	
            }
            else {
                altitude = std::numeric_limits<double>::quiet_NaN();
            }
            altitude_pub.publish(altitude_msg);
        }
    }
    lidarLite->closeLidarLite();
}
