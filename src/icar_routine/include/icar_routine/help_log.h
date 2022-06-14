#ifndef HELP_LOG_H_
#define HELP_LOG_H_

#include "boost/date_time.hpp"
#include "icar_miscellaneous/log.h"
#include "ros/ros.h"

class help_log
{
private:
    //=====Publisher
    ros::Publisher pub_log;

    //==================================

    void help(const char *header, const char *format, va_list arg)
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

public:
    help_log()
    {
    }

    ~help_log()
    {
    }

    //==================================

    void init(ros::NodeHandle *NH)
    {
        pub_log = NH->advertise<icar_miscellaneous::log>("log", 10);
    }

    //==================================

    void info(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        help(" INFO", format, args);
        va_end(args);
    }

    void warn(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        help(" WARN", format, args);
        va_end(args);
    }

    void error(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        help("ERROR", format, args);
        va_end(args);
    }

    void fatal(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        help("FATAL", format, args);
        va_end(args);
    }
};

#endif