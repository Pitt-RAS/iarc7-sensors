#include <sensor_msgs/Range.h>
#include <iarc7_msgs/Nano.h>
#include <iarc7_msgs/FlowVector.h>
#include <iarc7_msgs/Float64Stamped.h>
#include <iarc7_msgs/Float64ArrayStamped.h>
#include <iarc7_msgs/ESCCommand.h>
#include <ros/ros.h>

void distributeMessages(iarc7_msgs::Nano nano_info, ros::Publisher& opticalflow_pub, ros::Publisher& rangefinder_pub, ros::Publisher& long_range_pub,
                        ros::Publisher& battery_publisher);

void sendESCCommand(iarc7_msgs::Float64ArrayStamped esc_cmnds, ros::Publisher& esc_cmd_pub);

boost::function<void (const iarc7_msgs::Nano&)> callback;

boost::function<void (const iarc7_msgs::Float64ArrayStamped)> esc_callback;

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

    ros::Publisher esc_cmd_publisher = 
        nh.advertise<iarc7_msgs::Float64ArrayStamped>("Nano_pwm_targets", 0);

    boost::function<void (const iarc7_msgs::Nano&)> callback = 
    [&] (const iarc7_msgs::Nano& nano_info){
        distributeMessages(nano_info, opticalflow_pub, rangefinder_pub, long_range_pub,
            battery_publisher);
    };

    boost::function<void (const iarc7_msgs::Float64ArrayStamped&)> esc_callback = 
    [&] (const iarc7_msgs::Float64ArrayStamped& esc_cmnds){
        sendESCCommand(esc_cmnds, esc_cmd_publisher);
    };
    // & just means "look at all the objects already declared and pull the ones you need"
    ros::Subscriber Nano_sub =  nh.subscribe<iarc7_msgs::Nano>("nano_data",
                                0,
                                callback);

    ros::Subscriber esc_cmd_sub = nh.subscribe<iarc7_msgs::Float64ArrayStamped>("uav_direction_command",
                                0, 
                                esc_callback);

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
    long_range_msg.min_range = 0.5;
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
    if(nano_info.flow_board_offset > 0)
    {
    opticalflow_pub.publish(flow_msg);
    }

    if(short_range_msg.header.stamp.toSec() < long_range_msg.header.stamp.toSec()
        || flow_msg.header.stamp.toSec() < long_range_msg.header.stamp.toSec())
    {
        ROS_WARN("Nano_transcriber received messages out of order");
    }

    ros::Duration battery_offset = ros::Duration().fromNSec((int64_t)(nano_info.battery_offset * 1000));

    battery_voltage.data = nano_info.battery_voltage;
    battery_voltage.header.stamp = nano_info.msg_received + battery_offset;
    battery_publisher.publish(battery_voltage);

    return;
}

void sendESCCommand(iarc7_msgs::Float64ArrayStamped esc_cmnds, ros::Publisher& esc_cmd_pub)
{
    iarc7_msgs::ESCCommand esc_cmd;
    esc_cmd.motor1PWM = esc_cmnds.data[0]*255/13;
    esc_cmd.motor2PWM = esc_cmnds.data[1]*255/13;
    esc_cmd.motor3PWM = esc_cmnds.data[2]*255/13;
    esc_cmd.motor4PWM = esc_cmnds.data[3]*255/13;

    esc_cmd.ESCCommandStamp = esc_cmnds.header.stamp;

    esc_cmd_pub.publish(esc_cmd);

    return;
}
