#include <ros/ros.h>
#include "maxon_control/speed_control.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"maxon_control");
    ros::NodeHandle nodeHandle("~");

    maxon_control::SpeedControl speedControl(nodeHandle);


    dynamic_reconfigure::Server<maxon_control::ParametersConfig> server;
    dynamic_reconfigure::Server<maxon_control::ParametersConfig>::CallbackType f;

    f = boost::bind(&maxon_control::SpeedControl::callback_pid_reconfigure, &speedControl, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
