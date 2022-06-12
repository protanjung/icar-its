#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_broadcaster");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    AS.start();
    ros::waitForShutdown();
}
