#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16MultiArray.h"
#include "tf/tf.h"

#define M_PER_PULSE 0.0022934
#define PULSE_PER_M 436.0409399

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);

void cllbck_sub_rotary_encoder(const std_msgs::UInt16MultiArrayConstPtr &msg);
void cllbck_sub_gyroscope(const std_msgs::Float32ConstPtr &msg);

//=====Timer
ros::Timer tim_50hz;
//=====Subscriber
ros::Subscriber sub_stm32_rotary_encoder;
ros::Subscriber sub_stm32_gyroscope;
//=====Publisher
ros::Publisher pub_odom_twist;
ros::Publisher pub_odom_pose;
ros::Publisher pub_odom;

// Rotary endcoder
uint16_t rotary_encoder_kiri;
uint16_t rotary_encoder_kanan;

// Gyroscope
float gyroscope_deg;
float gyroscope_rad;

// Odometry
double tangential_velocity_kiri;
double tangential_velocity_kanan;
double vx = 0, vy = 0, vth = 0;
double x = 0, y = 0, th = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_interpreter");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    //=====Subscriber
    sub_stm32_rotary_encoder = NH.subscribe("/stm32_/rotary_encoder", 1, cllbck_sub_rotary_encoder);
    sub_stm32_gyroscope = NH.subscribe("/stm32_/gyroscope", 1, cllbck_sub_gyroscope);
    //=====Publisher
    pub_odom_twist = NH.advertise<geometry_msgs::Twist>("/odom/twist", 1);
    pub_odom_pose = NH.advertise<geometry_msgs::Pose>("/odom/pose", 1);
    pub_odom = NH.advertise<nav_msgs::Odometry>("/odom", 1);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
    static ros::Time time_start = ros::Time::now();
    static ros::Time time_now = ros::Time::now();

    static uint16_t prev_rotary_encoder_kiri = 0;
    static uint16_t prev_rotary_encoder_kanan = 0;
    static float prev_gyroscope_deg = 0;
    static float prev_gyroscope_rad = 0;

    time_now = ros::Time::now();

    if (time_now - time_start < ros::Duration(2))
    {
        prev_rotary_encoder_kiri = rotary_encoder_kiri;
        prev_rotary_encoder_kanan = rotary_encoder_kanan;
        prev_gyroscope_deg = gyroscope_deg;
        prev_gyroscope_rad = gyroscope_rad;
        return;
    }

    // Mengatasi diskontinuitas data rotary encoder kiri
    if (rotary_encoder_kiri < 16384 && prev_rotary_encoder_kiri > 49152)
        tangential_velocity_kiri = rotary_encoder_kiri + (65536 - prev_rotary_encoder_kiri);
    else if (rotary_encoder_kiri > 49152 && prev_rotary_encoder_kiri < 16384)
        tangential_velocity_kiri = -prev_rotary_encoder_kiri - (655356 - rotary_encoder_kiri);
    else
        tangential_velocity_kiri = rotary_encoder_kiri - prev_rotary_encoder_kiri;

    // Mengatasi diskontinuitas data rotary encoder kanan
    if (rotary_encoder_kanan < 16384 && prev_rotary_encoder_kanan > 49152)
        tangential_velocity_kanan = rotary_encoder_kanan + (65536 - prev_rotary_encoder_kanan);
    else if (rotary_encoder_kanan > 49152 && prev_rotary_encoder_kanan < 16384)
        tangential_velocity_kanan = -prev_rotary_encoder_kanan - (655356 - rotary_encoder_kanan);
    else
        tangential_velocity_kanan = rotary_encoder_kanan - prev_rotary_encoder_kanan;

    // Menghitung odometry menggunakan data kecepatan roda dan gyroscope
    vth = gyroscope_rad - prev_gyroscope_rad;
    th += vth;

    vx = ((tangential_velocity_kiri + tangential_velocity_kanan) / 2 * M_PER_PULSE) * cos(th);
    vy = ((tangential_velocity_kiri + tangential_velocity_kanan) / 2 * M_PER_PULSE) * sin(th);
    x += vx;
    y += vy;

    // Publish data kecepatan dan posisi
    geometry_msgs::Twist msg_odom_twist;
    msg_odom_twist.linear.x = vx;
    msg_odom_twist.linear.y = vy;
    msg_odom_twist.angular.z = vth;
    pub_odom_twist.publish(msg_odom_twist);

    geometry_msgs::Pose msg_odom_pose;
    msg_odom_pose.position.x = x;
    msg_odom_pose.position.y = y;
    msg_odom_pose.orientation = tf::createQuaternionMsgFromYaw(th);
    pub_odom_pose.publish(msg_odom_pose);

    nav_msgs::Odometry msg_odom;
    msg_odom.header.stamp = time_now;
    msg_odom.header.frame_id = "odom";
    msg_odom.child_frame_id = "base_link";
    msg_odom.pose.pose = msg_odom_pose;
    msg_odom.twist.twist = msg_odom_twist;
    pub_odom.publish(msg_odom);

    // Mencatat posisi terakhir roda kanan dan kiri
    prev_rotary_encoder_kiri = rotary_encoder_kiri;
    prev_rotary_encoder_kanan = rotary_encoder_kanan;
    prev_gyroscope_deg = gyroscope_deg;
    prev_gyroscope_rad = gyroscope_rad;
}

//==============================================================================

void cllbck_sub_rotary_encoder(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    rotary_encoder_kiri = msg->data[0];
    rotary_encoder_kanan = msg->data[1];
}

void cllbck_sub_gyroscope(const std_msgs::Float32ConstPtr &msg)
{
    gyroscope_deg = msg->data;
    gyroscope_rad = msg->data * M_PI / 180;
}