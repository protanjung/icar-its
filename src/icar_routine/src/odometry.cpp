#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16MultiArray.h"
#include "tf/tf.h"

#define M_PER_PULSE 0.0022934
#define PULSE_PER_M 436.0409399

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_stm32_to_pc_rotary_encoder(const std_msgs::UInt16MultiArrayConstPtr &msg);
void cllbck_sub_stm32_to_pc_gyroscope(const std_msgs::Float32ConstPtr &msg);

//=====Timer
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_stm32_to_pc_rotary_encoder;
ros::Subscriber sub_stm32_to_pc_gyroscope;
//=====Publisher
ros::Publisher pub_odom;

//-----Rotary encoder
//===================
uint16_t rotary_encoder_kiri;
uint16_t rotary_encoder_kanan;

//-----Gyroscope
//==============
float gyroscope_deg;
float gyroscope_rad;

//-----Odometry
//=============
double tangential_velocity_kiri;
double tangential_velocity_kanan;
double vx, vy, vth;
double x, y, th;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_stm32_to_pc_rotary_encoder = NH.subscribe("stm32_to_pc/rotary_encoder", 1, cllbck_sub_stm32_to_pc_rotary_encoder);
    sub_stm32_to_pc_gyroscope = NH.subscribe("stm32_to_pc/gyroscope", 1, cllbck_sub_stm32_to_pc_gyroscope);
    //=====Publisher
    pub_odom = NH.advertise<nav_msgs::Odometry>("odom", 1);

    AS.start();
    ros::waitForShutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
    static ros::Time time_start = ros::Time::now();
    static ros::Time time_now = ros::Time::now();

    static uint16_t prev_rotary_encoder_kiri;
    static uint16_t prev_rotary_encoder_kanan;
    static float prev_gyroscope_deg;
    static float prev_gyroscope_rad;

    time_now = ros::Time::now();

    if (time_now - time_start < ros::Duration(2))
    {
        prev_rotary_encoder_kiri = rotary_encoder_kiri;
        prev_rotary_encoder_kanan = rotary_encoder_kanan;
        prev_gyroscope_deg = gyroscope_deg;
        prev_gyroscope_rad = gyroscope_rad;

        return;
    }

    if (rotary_encoder_kiri < 16384 && prev_rotary_encoder_kiri > 49152)
        tangential_velocity_kiri = rotary_encoder_kiri + (65536 - prev_rotary_encoder_kiri);
    else if (rotary_encoder_kiri > 49152 && prev_rotary_encoder_kiri < 16384)
        tangential_velocity_kiri = -prev_rotary_encoder_kiri - (655356 - rotary_encoder_kiri);
    else
        tangential_velocity_kiri = rotary_encoder_kiri - prev_rotary_encoder_kiri;

    if (rotary_encoder_kanan < 16384 && prev_rotary_encoder_kanan > 49152)
        tangential_velocity_kanan = rotary_encoder_kanan + (65536 - prev_rotary_encoder_kanan);
    else if (rotary_encoder_kanan > 49152 && prev_rotary_encoder_kanan < 16384)
        tangential_velocity_kanan = -prev_rotary_encoder_kanan - (655356 - rotary_encoder_kanan);
    else
        tangential_velocity_kanan = rotary_encoder_kanan - prev_rotary_encoder_kanan;

    vth = gyroscope_rad - prev_gyroscope_rad;
    th += vth;

    vx = ((tangential_velocity_kiri + tangential_velocity_kanan) * 0.5 * M_PER_PULSE) * cos(th);
    vy = ((tangential_velocity_kiri + tangential_velocity_kanan) * 0.5 * M_PER_PULSE) * sin(th);
    x += vx;
    y += vy;

    geometry_msgs::Twist odom_twist;
    odom_twist.linear.x = vx;
    odom_twist.linear.y = vy;
    odom_twist.angular.z = vth;

    geometry_msgs::Pose odom_pose;
    odom_pose.position.x = x;
    odom_pose.position.y = y;
    odom_pose.orientation = tf::createQuaternionMsgFromYaw(th);

    nav_msgs::Odometry msg_odom;
    msg_odom.header.stamp = time_now;
    msg_odom.header.frame_id = "odom";
    msg_odom.child_frame_id = "base_link";
    msg_odom.twist.twist = odom_twist;
    msg_odom.pose.pose = odom_pose;
    pub_odom.publish(msg_odom);

    prev_rotary_encoder_kiri = rotary_encoder_kiri;
    prev_rotary_encoder_kanan = rotary_encoder_kanan;
    prev_gyroscope_deg = gyroscope_deg;
    prev_gyroscope_rad = gyroscope_rad;
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_stm32_to_pc_rotary_encoder(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    rotary_encoder_kiri = msg->data[0];
    rotary_encoder_kanan = msg->data[1];
}

void cllbck_sub_stm32_to_pc_gyroscope(const std_msgs::Float32ConstPtr &msg)
{
    gyroscope_deg = msg->data;
    gyroscope_rad = msg->data * M_PI / 180;
}