#include "icar_routine/routine.h"

//=====Timer
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_odom;
ros::Subscriber sub_stm32_to_pc_remote;
ros::Subscriber sub_stm32_to_pc_throttle_position;
ros::Subscriber sub_stm32_to_pc_steering_position;
//=====Publisher
ros::Publisher pub_throttle;
ros::Publisher pub_steering;
ros::Publisher pub_transmission;
//=====Miscellaneous
help_log _log;
help_joy _joy;
help_led _led;
help_rumble _rumble;

//-----Status Algoritma
//=====================
int status_algoritma = STATUS_IDDLE;
int status_sub_algoritma = STATUS_IDDLE;
int status_sub_sub_algoritma = STATUS_IDDLE;

//-----Odometry
//=============
double x = 0, y = 0, th = 0;

//-----STM32 to PC
//================
uint16_t remote[16] = {0};
int16_t throttle_position = 0;
int16_t steering_position = 0;

//-----DS4 to PC
//==============
icar_routine::ds4_to_pc_joy joy;
icar_routine::ds4_to_pc_joy prev_joy;

//-----Keyboard
//=============
uint8_t key;
uint8_t prev_key;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "routine");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_odom = NH.subscribe("odom", 1, cllbck_sub_odom);
    sub_stm32_to_pc_remote = NH.subscribe("stm32_to_pc/remote", 1, cllbck_sub_stm32_to_pc_remote);
    sub_stm32_to_pc_throttle_position = NH.subscribe("stm32_to_pc/throttle_position", 1, cllbck_sub_stm32_to_pc_throttle_position);
    sub_stm32_to_pc_steering_position = NH.subscribe("stm32_to_pc/steering_position", 1, cllbck_sub_stm32_to_pc_steering_position);
    //=====Publisher
    pub_throttle = NH.advertise<std_msgs::Int16>("stm32_from_pc/throttle", 1);
    pub_steering = NH.advertise<std_msgs::Int16>("stm32_from_pc/steering", 1);
    pub_transmission = NH.advertise<std_msgs::Int8>("stm32_from_pc/transmission", 1);
    //=====Miscellaneous
    _log.init(&NH);
    _joy.init(&NH);
    _led.init(&NH);
    _rumble.init(&NH);

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

void cllbck_sub_stm32_to_pc_remote(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    for (int i = 0; i < 16; i++)
        remote[i] = msg->data[i];
}

void cllbck_sub_stm32_to_pc_throttle_position(const std_msgs::Int16ConstPtr &msg)
{
    throttle_position = msg->data;
}

void cllbck_sub_stm32_to_pc_steering_position(const std_msgs::Int16ConstPtr &msg)
{
    steering_position = msg->data;
}

//------------------------------------------------------------------------------
//==============================================================================

int routine_init()
{
    ros::Duration(2).sleep();

    _log.warn(R"(
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
    baca_joystick();
    baca_keyboard();

    algorithm_routine();

    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

void baca_joystick()
{
    _joy.joy(joy, prev_joy);
}

void baca_keyboard()
{
    static bool initialized = false;
    static struct termios old_attribute;
    static struct termios new_attribute;
    static int old_f;
    static int new_f;

    if (!initialized)
    {
        initialized = true;

        tcgetattr(0, &old_attribute);
        tcgetattr(0, &new_attribute);
        new_attribute.c_lflag &= ~ICANON;
        new_attribute.c_lflag &= ~ECHO;

        old_f = fcntl(0, F_GETFL, 0);
        new_f = fcntl(0, F_GETFL, 0);
        new_f |= O_NONBLOCK;
    }

    tcsetattr(0, TCSANOW, &new_attribute);
    fcntl(0, F_SETFL, new_f);

    int ch = getchar();

    tcsetattr(0, TCSANOW, &old_attribute);
    fcntl(0, F_SETFL, old_f);

    prev_key = key;

    if (ch != EOF)
        key = ch;
    else
        key = 0;
}