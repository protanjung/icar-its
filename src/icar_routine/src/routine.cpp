#include "icar_hardware/ds4_led.h"
#include "icar_hardware/ds4_rumble.h"
#include "icar_miscellaneous/log.h"
#include "icar_routine/icar_routine.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);

int routine_init();
int routine_routine();

//=====Timer
ros::Timer tim_50hz;
//=====Publisher
ros::Publisher pub_ds4_led;
ros::Publisher pub_ds4_rumble;
ros::Publisher pub_log;

// Status Algoritma
int status_algoritma;
int status_sub_algoritma;
int status_sub_sub_algoritma;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "routine");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    //=====Publisher
    pub_ds4_led = NH.advertise<icar_hardware::ds4_led>("/ds4/led", 1);
    pub_ds4_rumble = NH.advertise<icar_hardware::ds4_rumble>("/ds4/rumble", 1);
    pub_log = NH.advertise<icar_miscellaneous::log>("/log", 10);

    if (routine_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
    if (routine_routine() == -1)
        ros::shutdown();
}

//==============================================================================

int routine_init()
{
    return 0;
}

int routine_routine()
{
    return 0;
}
