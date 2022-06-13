#include "icar_miscellaneous/log.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_sub_log(const icar_miscellaneous::logConstPtr &msg);

//=====Subscriber
ros::Subscriber sub_log;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "log");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Subscriber
    sub_log = NH.subscribe("/log", 10, cllbck_sub_log);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_sub_log(const icar_miscellaneous::logConstPtr &msg)
{
}