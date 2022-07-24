#ifndef PURE_PURSUIT_H_
#define PURE_PURSUIT_H_

#include "ros/ros.h"

class pure_pursuit
{
private:
    double _wheel_base;
    double _look_ahead_distance;

    std::vector<geometry_msgs::Point> _path;
    uint32_t _path_index;
    uint32_t _path_index_start;
    uint32_t _path_index_stop;

public:
    float x_goal, y_goal;
    float alpha;
    float delta;

    //----------------------------------
    //==================================

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
        if (path.size() == 0)
            return;

        _path = path;
        _path_index = 1;
        _path_index_start = 0;
        _path_index_stop = 1;
    }

    void routine(double x, double y, double th)
    {
        if (_path.size() == 0)
            return;

        // Mencari index waypoint yang harus
        // dilewati terlebih dahulu
        while (_path_index < _path.size())
        {
            double dx = _path[_path_index].x - x;
            double dy = _path[_path_index].y - y;

            if (sqrt(dx * dx + dy * dy) > _look_ahead_distance)
            {
                _path_index_start = _path_index - 1;
                _path_index_stop = _path_index - 0;
                break;
            }

            _path_index++;
        }

        // Jika sudah mencapai index waypoint terakhir
        // maka menggunakan index waypoint terakhir
        if (_path_index >= _path.size())
        {
            // TODO Ini di-looping terus menerus
            _path_index = 1;
            _path_index_start = 0;
            _path_index_stop = 1;

            // TODO Masih bingung ini mau diisi apa
            x_goal = _path[_path_index_stop].x;
            y_goal = _path[_path_index_stop].y;
        }
        // Jika belum mencapai index waypoint terakhir
        // maka menggunakan perpotongan lingkaran dan garis
        else if (_path_index < _path.size())
        {
            float x_stop_minus_x_start = _path[_path_index_stop].x - _path[_path_index_start].x;
            float y_stop_minus_y_start = _path[_path_index_stop].y - _path[_path_index_start].y;
            float x_start_minus_x = _path[_path_index_start].x - x;
            float y_start_minus_y = _path[_path_index_start].y - y;

            float a = x_stop_minus_x_start * x_stop_minus_x_start + y_stop_minus_y_start * y_stop_minus_y_start;
            float b = 2 * (x_start_minus_x * x_stop_minus_x_start + y_start_minus_y * y_stop_minus_y_start);
            float c = x_start_minus_x * x_start_minus_x + y_start_minus_y * y_start_minus_y - _look_ahead_distance * _look_ahead_distance;

            float discriminant = b * b - 4 * a * c;

            // Jika tidak ada perpotongan
            if (discriminant < 0)
            {
                // TODO Masih bingung ini mau diisi apa
                x_goal = _path[_path_index_stop].x;
                y_goal = _path[_path_index_stop].y;
            }
            // Jika ada perpotongan
            else
            {
                float t1 = (-b + sqrtf(discriminant)) / (2 * a);
                float t2 = (-b - sqrtf(discriminant)) / (2 * a);

                if (t1 >= 0 && t1 <= 1)
                {
                    x_goal = _path[_path_index_start].x + t1 * x_stop_minus_x_start;
                    y_goal = _path[_path_index_start].y + t1 * y_stop_minus_y_start;
                }
                else if (t2 >= 0 && t2 <= 1)
                {
                    x_goal = _path[_path_index_start].x + t2 * x_stop_minus_x_start;
                    y_goal = _path[_path_index_start].y + t2 * y_stop_minus_y_start;
                }
                else
                {
                    // TODO Masih bingung ini mau diisi apa
                    x_goal = _path[_path_index_stop].x;
                    y_goal = _path[_path_index_stop].y;
                }
            }
        }

        // Kontrol steering mobil untuk menuju
        // titik perpotongan garis dan lingkaran
        alpha = atan2f(y_goal - y, x_goal - x) - th;
        delta = atan2f(2 * _wheel_base * sinf(alpha), _look_ahead_distance);
    }

    //----------------------------------
    //==================================

    double get_wheel_base()
    {
        return _wheel_base;
    }

    double get_look_ahead_distance()
    {
        return _look_ahead_distance;
    }

    //----------------------------------
    //==================================

    void set_wheel_base(double wheel_base)
    {
        _wheel_base = wheel_base;
    }

    void set_look_ahead_distance(double look_ahead_distance)
    {
        _look_ahead_distance = look_ahead_distance;
    }
};

#endif