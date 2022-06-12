#include "ds4_driver/Feedback.h"
#include "ds4_driver/Status.h"
#include "icar_hardware/stm32_from_pc.h"
#include "icar_hardware/stm32_to_pc.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_profiler");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    AS.start();
    ros::waitForShutdown();
}
