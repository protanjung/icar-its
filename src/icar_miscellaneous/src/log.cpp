#include "icar_miscellaneous/log.h"
#include "boost/date_time.hpp"
#include "boost/filesystem.hpp"
#include "icar_miscellaneous/color.h"
#include "icar_miscellaneous/version.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_log(const icar_miscellaneous::logConstPtr &msg);

int log_init();
int log_routine();

std::string folder_to_make();
std::string folder_to_remove();
void make_folder(std::string path);
void remove_folder(std::string path);

//=====Timer
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_log;

//-----Log
//========
std::string log_path;
std::ofstream log_file;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "log");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_log = NH.subscribe("/log", 10, cllbck_sub_log);

    if (log_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
    if (log_routine() == -1)
        ros::shutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_log(const icar_miscellaneous::logConstPtr &msg)
{
    make_folder(folder_to_make());
    remove_folder(folder_to_remove());

    boost::posix_time::ptime time_ptime = boost::posix_time::microsec_clock::local_time();
    std::string time_string = boost::posix_time::to_simple_string(time_ptime);

    std::string log_datetime = color::rize(msg->datetime, "Green");
    std::string log_header = color::rize(msg->header, "Blue");

    std::string log_body;
    if (msg->header.find("INFO") != std::string::npos)
        log_body = color::rize(msg->body, "White");
    else if (msg->header.find("WARN") != std::string::npos)
        log_body = color::rize(msg->body, "Yellow");
    else if (msg->header.find("ERROR") != std::string::npos)
        log_body = color::rize(msg->body, "Red");
    else if (msg->header.find("FATAL") != std::string::npos)
        log_body = color::rize(msg->body, "Red");

    std::cout << log_datetime << " [" << log_header << "] " << log_body << std::endl;

    log_file.open(std::string(folder_to_make() + "/" + time_string.substr(0, 11) + ".txt").c_str(), std::ofstream::out | std::ofstream::app);
    log_file << msg->datetime << " [" << msg->header << "] " << msg->body << std::endl;
    log_file.close();
}

//------------------------------------------------------------------------------
//==============================================================================

int log_init()
{
    log_path = getenv("HOME") + std::string("/icar-its-data/log");

    if (!boost::filesystem::exists(log_path))
        boost::filesystem::create_directories(log_path);

    return 0;
}

int log_routine()
{
    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

std::string folder_to_make()
{
    boost::posix_time::ptime time_ptime = boost::posix_time::microsec_clock::local_time();
    std::string time_string = boost::posix_time::to_simple_string(time_ptime);

    return log_path + "/" + time_string.substr(0, 8);
}

std::string folder_to_remove()
{
    boost::posix_time::ptime time_ptime = boost::posix_time::microsec_clock::local_time() - boost::posix_time::hours(180 * 24);
    std::string time_string = boost::posix_time::to_simple_string(time_ptime);

    return log_path + "/" + time_string.substr(0, 8);
}

void make_folder(std::string path)
{
    if (!boost::filesystem::exists(path))
        boost::filesystem::create_directories(path);
}

void remove_folder(std::string path)
{
    if (boost::filesystem::exists(path))
        boost::filesystem::remove_all(path);
}