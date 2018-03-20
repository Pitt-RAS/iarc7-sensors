#include <sensor_msgs/Range.h>
#include <iarc7_msgs/Nano.h>
#include <iarc7_msgs/FlowVector.h>
#include <ros/ros.h>

void distributeMessages(iarc7_msgs::Nano nano_info, ros::Publisher& opticalflow_pub, ros::Publisher& rangefinder_pub, ros::Publisher& long_range_pub);

boost::function<void (const iarc7_msgs::Nano&)> callback;

int main(int argc, char* argv[]){
    
    ros::init(argc, argv, "Nano_transcriber");
    ros::NodeHandle nh;

    ROS_WARN("Nano transcriber beginning");

    ros::Publisher rangefinder_pub = 
        nh.advertise<sensor_msgs::Range>("short_distance_lidar", 0);

    ros::Publisher long_range_pub = 
        nh.advertise<sensor_msgs::Range>("long_distance_lidar", 0);

    ros::Publisher opticalflow_pub = 
        nh.advertise<iarc7_msgs::FlowVector>("flow_vector", 0); 

    boost::function<void (const iarc7_msgs::Nano&)> callback = 
    [&] (const iarc7_msgs::Nano& nano_info){
        distributeMessages(nano_info, opticalflow_pub, rangefinder_pub, long_range_pub);
    };
    // & just means "look at all the objects already declared and pull the ones you need"
    ros::Subscriber Nano_sub = nh.subscribe<iarc7_msgs::Nano>("nano_data",
                                 0,
                                 callback);
    ros::spin();
}


void distributeMessages(iarc7_msgs::Nano nano_info, 
                        ros::Publisher& opticalflow_pub,
                        ros::Publisher& rangefinder_pub,
                        ros::Publisher& long_range_pub){

//  ROS_WARN("We are distributing things - current time - %f", ros::Time::now().toSec());
    sensor_msgs::Range long_range_msg;
    sensor_msgs::Range short_range_msg;
    iarc7_msgs::FlowVector flow_msg;

    if(nano_info.identifier == 1){
//        ROS_WARN("Got a short range message");
        short_range_msg.range = nano_info.short_range;
        short_range_msg.min_range = 0.01;
        short_range_msg.max_range = 0.800;
        short_range_msg.header.stamp = nano_info.msg_received;
        short_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        short_range_msg.field_of_view = 0.3;
        short_range_msg.header.frame_id = "/short_distance_lidar";
        rangefinder_pub.publish(short_range_msg);
        return;
    }

    else if(nano_info.identifier == 2){
 //       ROS_WARN("Got a long range message");
        long_range_msg.range = nano_info.long_range;
        long_range_msg.min_range = 0.3;
        long_range_msg.max_range = 10;
        long_range_msg.header.stamp = nano_info.msg_received;
        long_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
        long_range_msg.field_of_view = 0.3;
        long_range_msg.header.frame_id = "/tfmini";
        long_range_pub.publish(long_range_msg);
        return;
    }

    else if(nano_info.identifier == 4){
 //       ROS_WARN("Got an optical flow message");
        flow_msg.header.stamp = nano_info.msg_received;
        flow_msg.deltaX = nano_info.deltaX;
        flow_msg.deltaY = nano_info.deltaY;

        opticalflow_pub.publish(flow_msg);
        return;
    }

}
