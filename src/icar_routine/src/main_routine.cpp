#include "icar_hardware/ds4_led.h"
#include "icar_hardware/ds4_rumble.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);

//=====Timer
ros::Timer tim_50hz;
//=====Publisher
ros::Publisher pub_ds4_led;
ros::Publisher pub_ds4_rumble;

// Status Algoritma
int status_algoritma;
int status_sub_algoritma;
int status_sub_sub_algoritma;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_routine");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    //=====Publisher
    pub_ds4_led = NH.advertise<icar_hardware::ds4_led>("/ds4_/led", 1);
    pub_ds4_rumble = NH.advertise<icar_hardware::ds4_rumble>("/ds4_/rumble", 1);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
}