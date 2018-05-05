#include <sensor_msgs/Range.h>
#include <iarc7_msgs/Nano.h>
#include <iarc7_msgs/FlowVector.h>
#include <iarc7_msgs/Float64Stamped.h>
#include <iarc7_msgs/PlanarThrottle.h>
#include <iarc7_msgs/ESCCommand.h>
#include <iarc7_msgs/OrientationThrottleStamped.h>
#include <iarc7_msgs/FlightControllerStatus.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

void distributeMessages(iarc7_msgs::Nano nano_info, 
                        ros::Publisher& opticalflow_pub,
                        ros::Publisher& rangefinder_pub,
                        ros::Publisher& long_range_pub,
                        ros::Publisher& battery_publisher){

    // Each of the "offset" variables is multiplied by 10000 to convert from the tens 
    // of microseconds offsets given to us by the AVR chip to nanoseconds.

    sensor_msgs::Range long_range_msg;
    sensor_msgs::Range short_range_msg;
    iarc7_msgs::FlowVector flow_msg;
    iarc7_msgs::Float64Stamped battery_voltage;

    long_range_msg.range = nano_info.long_range;
    long_range_msg.min_range = 0.5;
    long_range_msg.max_range = 10;
    ros::Duration long_range_offset = ros::Duration().fromNSec(((int64_t)nano_info.long_range_offset * 10000));
    long_range_msg.header.stamp = nano_info.msg_received - long_range_offset;
    long_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    long_range_msg.field_of_view = 0.3;
    long_range_msg.header.frame_id = "/tfmini";

    // From here on out, we check that the offsets are not 0 because the firmware will set the time offset of a 
    // measurement to 0 if the sensor is not yet ready to deliver a measurement.
    if(nano_info.long_range_offset > 0) {
        long_range_pub.publish(long_range_msg);
    }

    short_range_msg.range = nano_info.short_range;
    short_range_msg.min_range = 0.01;
    short_range_msg.max_range = 0.800;
    ros::Duration short_range_offset = ros::Duration().fromNSec(((int64_t)nano_info.short_range_offset * 10000));
    short_range_msg.header.stamp =  nano_info.msg_received - short_range_offset;
    short_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    short_range_msg.field_of_view = 0.3;
    short_range_msg.header.frame_id = "/short_distance_lidar";
    if(nano_info.short_range_offset > 0)
    {
        rangefinder_pub.publish(short_range_msg);
    }
    
    ros::Duration flow_offset = ros::Duration().fromNSec(((int64_t)nano_info.short_range_offset * 10000));
    flow_msg.header.stamp = (nano_info.msg_received - flow_offset);
    flow_msg.deltaX = nano_info.deltaX;
    flow_msg.deltaY = nano_info.deltaY;
    if(nano_info.flow_board_offset > 0)
    {
        opticalflow_pub.publish(flow_msg);
    }

    battery_voltage.data = nano_info.battery_voltage;
    battery_voltage.header.stamp = nano_info.msg_received;
    battery_publisher.publish(battery_voltage);

    return;
}

void sendESCCommand(iarc7_msgs::PlanarThrottle esc_cmnds,
                    ros::Publisher& esc_cmd_pub,
                    bool activateSideRotors)
{

    iarc7_msgs::ESCCommand esc_cmd;

    // A pulse width of 125 ms corresponds to no thrust, and 250 ms is 100% thrust.
    // These pulses are sent directly from the AVR chip to the side rotors.
    if(activateSideRotors)
    {
        esc_cmd.front_motor_PWM = (esc_cmnds.front_throttle*125) + 125;
        esc_cmd.back_motor_PWM = (esc_cmnds.back_throttle*125) + 125;
        esc_cmd.left_motor_PWM = (esc_cmnds.left_throttle*125) + 125;
        esc_cmd.right_motor_PWM = (esc_cmnds.right_throttle*125) + 125;
    }
    else {
        esc_cmd.front_motor_PWM = 125;
        esc_cmd.back_motor_PWM = 125;
        esc_cmd.left_motor_PWM = 125;
        esc_cmd.right_motor_PWM = 125;
    }

    esc_cmd_pub.publish(esc_cmd);

    return;

}

int main(int argc, char* argv[]){
    
    ros::init(argc, argv, "nano_transcriber");
    ros::NodeHandle nh;

    ros::Publisher rangefinder_pub = 
        nh.advertise<sensor_msgs::Range>("short_distance_lidar", 0);

    ros::Publisher long_range_pub = 
        nh.advertise<sensor_msgs::Range>("long_distance_lidar", 0);

    ros::Publisher opticalflow_pub = 
        nh.advertise<iarc7_msgs::FlowVector>("flow_vector", 0); 

    ros::Publisher battery_publisher = 
        nh.advertise<iarc7_msgs::Float64Stamped>("motor_battery", 0);

    ros::Publisher esc_cmd_publisher = 
        nh.advertise<iarc7_msgs::ESCCommand>("esc_commands", 0);

    // Lambdas that set up callbacks for subsribers to the AVR chip, the flight controller,
    // and the side rotor commands
    boost::function<void (const iarc7_msgs::Nano&)> callback = 
    [&] (const iarc7_msgs::Nano& nano_info){
        distributeMessages(nano_info, opticalflow_pub, rangefinder_pub, long_range_pub,
            battery_publisher);
    };

    bool activateSideRotors = false;
    bool joystickActive = false;

    boost::function<void (const iarc7_msgs::OrientationThrottleStamped&)> esc_callback = 
    [&] (const iarc7_msgs::OrientationThrottleStamped& uav_cmd){
        if(!joystickActive) {
            sendESCCommand(uav_cmd.planar, esc_cmd_publisher, activateSideRotors);
        }
    };

    boost::function<void (const iarc7_msgs::FlightControllerStatus&)> fc_callback = 
    [&] (const iarc7_msgs::FlightControllerStatus& fc_status){
        activateSideRotors = fc_status.armed && fc_status.auto_pilot;
    };

    boost::function<void (const sensor_msgs::Joy&)> joy_callback = 
    [&] (const sensor_msgs::Joy& joy){
        joystickActive = joy.buttons[5];
        iarc7_msgs::PlanarThrottle planar_throttle;

        planar_throttle.front_throttle = std::max(std::min(-joy.axes[1], 1.0f), 0.0f);
        planar_throttle.back_throttle = std::max(std::min(joy.axes[1], 1.0f), 0.0f);
        planar_throttle.right_throttle = std::max(std::min(joy.axes[0], 1.0f), 0.0f);
        planar_throttle.left_throttle = std::max(std::min(-joy.axes[0], 1.0f), 0.0f);

        if(joystickActive) {
            sendESCCommand(planar_throttle, esc_cmd_publisher, true);
        }
    };

    ros::Subscriber Nano_sub =  nh.subscribe<iarc7_msgs::Nano>("/nano_data",
                                0,
                                callback);

    ros::Subscriber esc_cmd_sub = nh.subscribe<iarc7_msgs::OrientationThrottleStamped>("/uav_direction_command",
                                0, 
                                esc_callback);

    ros::Subscriber joy_cmd_sub = nh.subscribe<sensor_msgs::Joy>("/joy",
                                0, 
                                joy_callback);

    ros::Subscriber fc_status_sub = nh.subscribe<iarc7_msgs::FlightControllerStatus>("/fc_status",
                                  0,
                                  fc_callback);

    ros::spin();

}
