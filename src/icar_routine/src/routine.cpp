#include "icar_routine/routine.h"

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_odom(const nav_msgs::OdometryConstPtr &msg);

int routine_init();
int routine_routine();

//=====Timer
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_odom;
//=====Miscellaneous
miscellaneous_log misc_log;

//-----Status Algoritma
//=====================
int status_algoritma = STATUS_IDDLE;
int status_sub_algoritma = STATUS_IDDLE;
int status_sub_sub_algoritma = STATUS_IDDLE;

//-----Odometry
//=============
double x, y, th;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "routine");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    ros::Subscriber sub_odom = NH.subscribe("odom", 1, cllbck_sub_odom);
    //=====Miscellaneous
    misc_log.init(&NH);

    if (routine_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
    if (routine_routine() == -1)
        ros::shutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_odom(const nav_msgs::OdometryConstPtr &msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    th = tf::getYaw(msg->pose.pose.orientation);
}

//------------------------------------------------------------------------------
//==============================================================================

int routine_init()
{
    ros::Duration(2).sleep();

    misc_log.warn(R"(
 __________________________
/         ICAR ITS         \
\ Pandu Marin Krisna Zidan /
 --------------------------
    \
     \
       /\   /\
      //\\_//\\     ____
      \_     _/    /   /
       / - * \    /^^^]
       \_\O/_/    [   ]
        /   \_    [   /
        \     \_  /  /
         [ [ /  \/ _/
        _[ [ \  /_/
)");

    return 0;
}

int routine_routine()
{
    return 0;
}