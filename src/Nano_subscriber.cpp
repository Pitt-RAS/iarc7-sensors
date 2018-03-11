#include <sensor_msgs/Range.h>
#include <iarc7_msgs/Nano.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Nano_publisher");
    ros::NodeHandle nh;

    sensor_msgs::Range range_msg;
    ros::Publisher rangefinder_pub = 
        nh.advertise<sensor_msgs::Range>("short_distance_lidar", 0);

    iarc7_msgs::FlowVector flow_msg;
    ros::Publisher opticalflow_pub = 
        nh.advertise<iarc7_msgs::FlowVector>("flow_vector", 0);

    iarc7_msgs::Nano nano_info; 
    ros::Subscriber Nano_sub("nano_data", &nano_info);
   
    range_msg.header = nano_info.short_range_header;
    flow_msg.header = nano_info.optical_header;

    range_msg.min_range = nano_info.short_min_range;
    range_msg.max_range = nano_info.short_max_range;
    range_msg.range = nano_info.short_range;

    flow_msg.deltaX = nano_info.deltaX;
    flow_msg.deltaY = nano_info.deltaY;

    opticalflow_pub.publish(flow_msg);
    rangefinder_pub.publish(range_msg);


    ros::spin();
}    
