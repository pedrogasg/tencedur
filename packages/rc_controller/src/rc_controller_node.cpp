#include <ros/ros.h>
#include "rc_controller/rc_controller.h"

int main(int argc, char **argv)
{
    ROS_INFO("Starting rc_controller ROS node...");

    ros::init(argc, argv, "rc_controller");
    ros::NodeHandle nh;

    rc_controller::RCController controller;
    if(!controller.init(nh))
    {
        ROS_ERROR("Could not initialize RCController node!");
        return -1;
    }

    if(!controller.arm())
    {
        ROS_ERROR("Could not arm Robot FCU!");
        return -1;
    }

    ROS_INFO("Rc_controller waiting for commands...");
    controller.spin();

    return 0;
}
