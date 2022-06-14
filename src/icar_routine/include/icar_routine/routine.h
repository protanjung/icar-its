#ifndef ROUTINE_H_
#define ROUTINE_H_

#include "fcntl.h"
#include "icar_routine/help_joy.h"
#include "icar_routine/help_led.h"
#include "icar_routine/help_log.h"
#include "icar_routine/help_rumble.h"
#include "nav_msgs/Odometry.h"
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

#define STATUS_IDDLE 0

#define STATUS_PATH_RECORD 1
#define STATUS_PATH_RECORD_IDDLE 0
#define STATUS_PATH_RECORD_STARTED 1
#define STATUS_PATH_RECORD_STOPPED 2

#define STATUS_TEST 9999

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
//=====Miscellaneous
extern help_log _log;
extern help_joy _joy;
extern help_led _led;
extern help_rumble _rumble;

//-----Status Algoritma
//=====================
extern int status_algoritma;
extern int status_sub_algoritma;
extern int status_sub_sub_algoritma;

//-----Odometry
//=============
extern double x, y, th;

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
extern uint8_t keyboard;
extern uint8_t prev_keyboard;

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_odom(const nav_msgs::OdometryConstPtr &msg);
void cllbck_sub_stm32_to_pc_remote(const std_msgs::UInt16MultiArrayConstPtr &msg);
void cllbck_sub_stm32_to_pc_throttle_position(const std_msgs::Int16ConstPtr &msg);
void cllbck_sub_stm32_to_pc_steering_position(const std_msgs::Int16ConstPtr &msg);

int routine_init();
int routine_routine();

void baca_joystick();
void baca_keyboard();

void algorithm_routine();

//=====Prototype - Motion
void jalan_manual(float _throttle, float _steering, uint8_t _transmission);

#endif