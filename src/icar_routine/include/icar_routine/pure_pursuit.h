#ifndef PURE_PURSUIT_H_
#define PURE_PURSUIT_H_

#include "ros/ros.h"

class pure_pursuit
{
private:
    double _wheel_base;
    double _look_ahead_distance;

    std::vector<geometry_msgs::Point> _path;

    uint64_t path_index;

    double x_goal;
    double y_goal;

public:
    pure_pursuit(double wheel_base, double look_ahead_distance)
    {
        _wheel_base = wheel_base;
        _look_ahead_distance = look_ahead_distance;
    }

    ~pure_pursuit()
    {
    }

    //----------------------------------
    //==================================

    void init(std::vector<geometry_msgs::Point> path)
    {
        _path = path;

        path_index = 0;

        x_goal = _path[path_index].x;
        y_goal = _path[path_index].y;
    }

    void routine(double x, double y, double _th)
    {
        // Mencari index waypoint yang harus
        // dilewati terlebih dahulu
        while (path_index++ < _path.size())
        {
            double dx = _path[path_index].x - x;
            double dy = _path[path_index].y - y;

            if (sqrt(dx * dx + dy * dy) > _look_ahead_distance)
                break;
        }

        // Jika mencapai index waypoint terakhir
        // maka menghentikan algoritma
        if (path_index >= _path.size())
        {
        }
    }
};

#endif