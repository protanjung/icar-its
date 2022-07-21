#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_odom(const nav_msgs::OdometryConstPtr &msg);

void send_transform(tfScalar x, tfScalar y, tfScalar z, tfScalar roll, tfScalar pitch, tfScalar yaw, std::string frame_id, std::string child_id);

//=====Timer
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_odom;

//-----Odometry
//=============
bool odom_is_ready = false;
nav_msgs::Odometry odom;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_broadcaster");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_odom = NH.subscribe("odom", 1, cllbck_sub_odom);

    AS.start();
    ros::waitForShutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
    if (odom_is_ready)
    {
        double x = odom.pose.pose.position.x;
        double y = odom.pose.pose.position.y;
        double th = tf::getYaw(odom.pose.pose.orientation);

        send_transform(x, y, 0,
                       0, 0, th * 180 / M_PI,
                       odom.header.frame_id, odom.child_frame_id);
    }

    //==================================

    send_transform(0.00, 0.00, 0.00,
                   0.00, 0.00, 0.00,
                   "base_link", "rear_axle_link");
    send_transform(2.10, 0.00, 0.00,
                   0.00, 0.00, 0.00,
                   "base_link", "front_axle_link");
    send_transform(2.85, 0.00, 0.65,
                   1.23, 20.00, 0.00,
                   "base_link", "lidar_link");
    send_transform(2.85, 0.00, 0.95,
                   0.00, 0.00, 0.00,
                   "base_link", "camera_link");
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_odom(const nav_msgs::OdometryConstPtr &msg)
{
    odom_is_ready = true;
    odom = *msg;
}

//------------------------------------------------------------------------------
//==============================================================================

void send_transform(tfScalar x, tfScalar y, tfScalar z, tfScalar roll, tfScalar pitch, tfScalar yaw, std::string frame_id, std::string child_id)
{
    static tf::TransformBroadcaster transform_broadcaster;

    tf::Vector3 origin;
    tf::Quaternion rotation;

    origin.setValue(x, y, z);
    rotation.setRPY(roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(rotation);

    transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_id));
}