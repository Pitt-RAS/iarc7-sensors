#include <iarc7_sensors/AltimeterFilter.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "altimeter_from_subscription");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string altitude_frame;
    private_nh.param("output_frame", altitude_frame, std::string(""));

    double altitude_variance;
    private_nh.param("altitude_variance", altitude_variance, 0.0);

    iarc7_sensors::AltimeterFilter filter(
            nh,
            altitude_frame,
            [=](double) -> double { return altitude_variance; },
            "level_quad");

    ros::spin();
}
