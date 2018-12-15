#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <smartAGV/smartBOT.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "smartrobot");

    ros::NodeHandle n;
    SmartBOT agvBot;

    agvBot.initialize(n);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}