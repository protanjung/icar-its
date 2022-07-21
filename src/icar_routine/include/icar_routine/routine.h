#ifndef ROUTINE_H_
#define ROUTINE_H_

#include "boost/filesystem.hpp"
#include "fcntl.h"
#include "geometry_msgs/Point.h"
#include "icar_routine/bicycle_model.h"
#include "icar_routine/help_joy.h"
#include "icar_routine/help_led.h"
#include "icar_routine/help_log.h"
#include "icar_routine/help_marker.h"
#include "icar_routine/help_rumble.h"
#include "icar_routine/pure_pursuit.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt16MultiArray.h"
#include "sys/ioctl.h"
#include "termios.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

#define STATUS_IDDLE 0
#define STATUS_ROUTE_RECORD 1
#define STATUS_ROUTE_LOAD 2

#define STATUS_ROUTE_RECORD_IDDLE STATUS_IDDLE
#define STATUS_ROUTE_RECORD_STARTED 1
#define STATUS_ROUTE_RECORD_STOPPED 2

#define STATUS_ROUTE_LOAD_IDDLE STATUS_IDDLE
#define STATUS_ROUTE_LOAD_LIST 1
#define STATUS_ROUTE_LOAD_SELECT 2
#define STATUS_ROUTE_LOAD_PROCESSED 3

//=====Timer
extern ros::Timer tim_100hz;
//=====Subscriber
extern ros::Subscriber sub_odom;
extern ros::Subscriber sub_stm32_to_pc_remote;
extern ros::Subscriber sub_stm32_to_pc_throttle_position;
extern ros::Subscriber sub_stm32_to_pc_steering_position;
//=====Publisher
extern ros::Publisher pub_throttle;
extern ros::Publisher pub_steering;
extern ros::Publisher pub_transmission;
//=====TransformListener
extern tf::TransformListener *transform_listener;
//=====Miscellaneous
extern help_log _log;
extern help_joy _joy;
extern help_led _led;
extern help_rumble _rumble;
extern help_marker _marker;

//-----Status Algoritma
//=====================
extern int status_algoritma;
extern int status_sub_algoritma;
extern int status_sub_sub_algoritma;

//-----Odometry
//=============
extern bool odom_is_ready;
extern nav_msgs::Odometry odom;

extern double x, y, th;
extern double x_rear, y_rear, th_rear;
extern double x_front, y_front, th_front;

//-----STM32 to PC
//================
extern uint16_t remote[16];
extern int16_t throttle_position;
extern int16_t steering_position;

//-----DS4 to PC
//==============
extern icar_routine::ds4_to_pc_joy joy;
extern icar_routine::ds4_to_pc_joy prev_joy;

//-----Keyboard
//=============
extern uint8_t key;
extern uint8_t prev_key;

//-----Metric
//===========
extern double velocity;
extern double acceleration;
extern double jerk;

//-----Folder Path
//================
extern std::string route_path;
extern std::string parameter_path;

//-----Route
//==========
extern std::fstream route_file;

//-----Marker
//===========
extern std::vector<geometry_msgs::Point> marker_odometry_front;
extern std::vector<geometry_msgs::Point> marker_odometry_rear;
extern std::vector<geometry_msgs::Point> marker_route_front;
extern std::vector<geometry_msgs::Point> marker_route_rear;

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_odom(const nav_msgs::OdometryConstPtr &msg);
void cllbck_sub_stm32_to_pc_remote(const std_msgs::UInt16MultiArrayConstPtr &msg);
void cllbck_sub_stm32_to_pc_throttle_position(const std_msgs::Int16ConstPtr &msg);
void cllbck_sub_stm32_to_pc_steering_position(const std_msgs::Int16ConstPtr &msg);

//=====Prototype - Common
int routine_init();
int routine_routine();

void baca_joystick();
void baca_keyboard();
void baca_metric();

//=====Prototype - Algorithm
void marker_odometry_routine();
void marker_route_routine();

void algorithm_reset();
void algorithm_routine();

void iddle_routine();
void record_route_routine();
void load_route_routine();

//=====Prototype - Motion
void jalan_manual(float _throttle, float _steering, uint8_t _transmission);

#endif