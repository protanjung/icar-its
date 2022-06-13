#ifndef MISCELLANEOUS_H_
#define MISCELLANEOUS_H_

#include "boost/date_time.hpp"
#include "icar_miscellaneous/log.h"
#include "ros/ros.h"

class miscellaneous_log
{
private:
    void help(const char *header, const char *format, va_list arg);

    ros::Publisher pub_log;

public:
    miscellaneous_log();
    ~miscellaneous_log();

    void init(ros::NodeHandle *NH);

    void info(const char *format, ...);
    void warn(const char *format, ...);
    void error(const char *format, ...);
    void fatal(const char *format, ...);
};

//------------------------------------------------------------------------------
//==============================================================================

miscellaneous_log::miscellaneous_log()
{
}

miscellaneous_log::~miscellaneous_log()
{
}

//------------------------------------------------------------------------------
//==============================================================================

void miscellaneous_log::help(const char *header, const char *format, va_list arg)
{
    char body[1024];
    vsnprintf(body, sizeof(body), format, arg);

    boost::posix_time::ptime time_ptime = boost::posix_time::microsec_clock::local_time();
    std::string time_string = boost::posix_time::to_simple_string(time_ptime);

    icar_miscellaneous::log msg_log;
    msg_log.datetime = time_string.substr(12, 12);
    msg_log.header = std::string(header);
    msg_log.body = std::string(body);
    pub_log.publish(msg_log);
}

//------------------------------------------------------------------------------
//==============================================================================

void miscellaneous_log::init(ros::NodeHandle *NH)
{
    pub_log = NH->advertise<icar_miscellaneous::log>("/log", 10);
}

//------------------------------------------------------------------------------
//==============================================================================

void miscellaneous_log::info(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    help(" INFO", format, args);
    va_end(args);
}

void miscellaneous_log::warn(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    help(" WARN", format, args);
    va_end(args);
}

void miscellaneous_log::error(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    help("ERROR", format, args);
    va_end(args);
}

void miscellaneous_log::fatal(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    help("FATAL", format, args);
    va_end(args);
}

#endif