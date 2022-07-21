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
//=====TransformListener
tf::TransformListener *transform_listener;
//=====Miscellaneous
help_log _log;
help_joy _joy;
help_led _led;
help_rumble _rumble;
help_marker _marker;

//-----Status Algoritma
//=====================
int status_algoritma = STATUS_IDDLE;
int status_sub_algoritma = STATUS_IDDLE;
int status_sub_sub_algoritma = STATUS_IDDLE;

//-----Odometry
//=============
bool odom_is_ready = false;
nav_msgs::Odometry odom;

double x = 0, y = 0, th = 0;
double x_rear = 0, y_rear = 0, th_rear = 0;
double x_front = 0, y_front = 0, th_front = 0;

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

//-----Metric
//===========
double velocity = 0;     // Dalam km/h
double acceleration = 0; // Dalam km/h^2
double jerk = 0;         // Dalam km/h^3

//-----Folder Path
//================
std::string route_path;
std::string parameter_path;

//-----Route
//==========
std::fstream route_file;

//-----Marker
//===========
std::vector<geometry_msgs::Point> marker_odometry_front;
std::vector<geometry_msgs::Point> marker_odometry_rear;
std::vector<geometry_msgs::Point> marker_route_front;
std::vector<geometry_msgs::Point> marker_route_rear;

//-----Pure Pursuit
//=================
pure_pursuit pp(2.1, 4.2);

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
    //=====TransformListener
    transform_listener = new tf::TransformListener(NH);
    //=====Miscellaneous
    _log.init(&NH);
    _joy.init(&NH);
    _led.init(&NH);
    _rumble.init(&NH);
    _marker.init(&NH);

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
    odom_is_ready = true;
    odom = *msg;

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    th = tf::getYaw(msg->pose.pose.orientation);

    transform_listener->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(0.1));

    tf::StampedTransform transform;

    transform_listener->lookupTransform("odom", "front_axle_link", ros::Time(0), transform);
    x_front = transform.getOrigin().x();
    y_front = transform.getOrigin().y();
    th_front = tf::getYaw(transform.getRotation());

    transform_listener->lookupTransform("odom", "rear_axle_link", ros::Time(0), transform);
    x_rear = transform.getOrigin().x();
    y_rear = transform.getOrigin().y();
    th_rear = tf::getYaw(transform.getRotation());
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

    route_path = getenv("HOME") + std::string("/icar-its-data/route");
    parameter_path = getenv("HOME") + std::string("/icar-its-data/parameter");

    if (!boost::filesystem::exists(route_path))
        boost::filesystem::create_directories(route_path);
    if (!boost::filesystem::exists(parameter_path))
        boost::filesystem::create_directories(parameter_path);

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
    baca_metric();

    algorithm_routine();

    marker_odometry_routine();
    marker_route_routine();

    if (!prev_joy.circle && joy.circle)
    {
        _log.warn("CIRCLE");
        pp.init(marker_route_rear);
    }

    if (!prev_joy.square && joy.square)
    {
        _log.warn("SQUARE");
        pp.routine(x, y, th);
    }

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

void baca_metric()
{
    static ros::Time time_last;
    static ros::Time time_now;

    static double prev_velocity;
    static double prev_acceleration;
    static double prev_x;
    static double prev_y;

    time_now = ros::Time::now();

    if (time_now - time_last > ros::Duration(0.05))
    {
        double dx = x - prev_x;
        double dy = y - prev_y;

        double distance = sqrt(dx * dx + dy * dy);
        double forward = cos(th - atan2(dy, dx));

        velocity = (distance * forward) / (time_now - time_last).toSec() * 3.6;
        acceleration = (velocity - prev_velocity) / (time_now - time_last).toSec() * 3.6;
        jerk = (acceleration - prev_acceleration) / (time_now - time_last).toSec() * 3.6;

        prev_velocity = velocity;
        prev_acceleration = acceleration;
        prev_x = x;
        prev_y = y;

        time_last = time_now;
    }
}