#ifndef ROUTINE_H_
#define ROUTINE_H_

#include "icar_routine/miscellaneous_log.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/tf.h"

//-----Status Algoritma
//=====================
#define STATUS_IDDLE 0

#define STATUS_PATH_RECORD 1
#define STATUS_PATH_RECORD_IDDLE 0
#define STATUS_PATH_RECORD_STARTED 1
#define STATUS_PATH_RECORD_STOPPED 2

#define STATUS_TEST 9999

#endif