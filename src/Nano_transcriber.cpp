#include <sensor_msgs/Range.h>
#include <iarc7_msgs/Nano.h>
#include <iarc7_msgs/FlowVector.h>
#include <iarc7_msgs/Float64Stamped.h>
#include <ros/ros.h>

void distributeMessages(iarc7_msgs::Nano nano_info, ros::Publisher& opticalflow_pub, ros::Publisher& rangefinder_pub, ros::Publisher& long_range_pub,
                        ros::Publisher& battery_publisher);

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

    ros::Publisher battery_publisher = 
        nh.advertise<iarc7_msgs::Float64Stamped>("motor_battery", 0);

    boost::function<void (const iarc7_msgs::Nano&)> callback = 
    [&] (const iarc7_msgs::Nano& nano_info){
        distributeMessages(nano_info, opticalflow_pub, rangefinder_pub, long_range_pub,
            battery_publisher);
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
                        ros::Publisher& long_range_pub,
                        ros::Publisher& battery_publisher){

    ros::Duration offsets;

    //ROS_WARN("We are distributing things - current time - %f", ros::Time::now().toSec());
    sensor_msgs::Range long_range_msg;
    sensor_msgs::Range short_range_msg;
    iarc7_msgs::FlowVector flow_msg;
    iarc7_msgs::Float64Stamped battery_voltage;

    long_range_msg.range = nano_info.long_range;
    long_range_msg.min_range = 0.3;
    long_range_msg.max_range = 10;
    long_range_msg.header.stamp = nano_info.msg_received;
    long_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    long_range_msg.field_of_view = 0.3;
    long_range_msg.header.frame_id = "/tfmini";
    long_range_pub.publish(long_range_msg);

    //ROS_WARN("The short range is %f", short_range_msg.range);
    short_range_msg.range = nano_info.short_range;
    short_range_msg.min_range = 0.01;
    short_range_msg.max_range = 0.800;

    ros::Duration short_range_offset = ros::Duration().fromNSec((int64_t)(nano_info.short_range_offset * 1000));
    short_range_msg.header.stamp =  nano_info.msg_received + short_range_offset;

    short_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    short_range_msg.field_of_view = 0.3;
    short_range_msg.header.frame_id = "/short_distance_lidar";

    // The actual short range lidar has a chance of not being ready when we poke it. In that case,
    // we do not want to republish.
    if(nano_info.short_range_offset > 0)
    {

        rangefinder_pub.publish(short_range_msg);

    }
    
    ros::Duration flow_offset = ros::Duration().fromNSec((int64_t)(nano_info.short_range_offset * 1000));

    flow_msg.header.stamp = (nano_info.msg_received + flow_offset);
    flow_msg.deltaX = nano_info.deltaX;
    flow_msg.deltaY = nano_info.deltaY;

    opticalflow_pub.publish(flow_msg);

    if(short_range_msg.header.stamp.toSec() < long_range_msg.header.stamp.toSec()
        || flow_msg.header.stamp.toSec() < long_range_msg.header.stamp.toSec())
    {
        ROS_WARN("Nano_transcriber received messages out of order");
    }

    ros::Duration battery_offset = ros::Duration().fromNSec((int64_t)(nano_info.battery_offset * 1000));

    battery_voltage.data = nano_info.battery_voltage;
    battery_voltage.header.stamp = nano_info.msg_received + battery_offset;
    battery_publisher.publish(battery_voltage);

    ROS_WARN("%f, %f, %f, %f\n",long_range_msg.header.stamp.toSec(),short_range_msg.header.stamp.toSec(), flow_msg.header.stamp.toSec(), battery_voltage.header.stamp.toSec());

    return;

}
