#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface_px4");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    AS.start();
    ros::waitForShutdown();
}
