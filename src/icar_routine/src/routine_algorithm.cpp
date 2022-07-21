#include "icar_routine/routine.h"

void marker_odometry_routine()
{
    static double prev_x = 0;
    static double prev_y = 0;

    double dx = x - prev_x;
    double dy = y - prev_y;

    if (sqrt(dx * dx + dy * dy) >= 0.01)
    {
        prev_x = x;
        prev_y = y;

        //==============================

        geometry_msgs::Point p;

        p.x = x_front;
        p.y = y_front;
        marker_odometry_front.push_back(p);
        _marker.add_line_strip(marker_odometry_front, "odom", "odometry", 0, 1.0, 0.4, 0.0, 0.05);

        p.x = x_rear;
        p.y = y_rear;
        marker_odometry_rear.push_back(p);
        _marker.add_line_strip(marker_odometry_rear, "odom", "odometry", 1, 0.8, 0.9, 0.1, 0.05);
    }
}

void marker_route_routine()
{
    if (marker_route_front.size() >= 2)
        _marker.add_line_strip(marker_route_front, "odom", "route", 0, 0.7, 0.9, 0.8, 0.05);

    if (marker_route_rear.size() >= 2)
        _marker.add_line_strip(marker_route_rear, "odom", "route", 1, 0.3, 0.0, 0.6, 0.05);
}

//------------------------------------------------------------------------------
//==============================================================================

void algorithm_reset()
{
    status_algoritma = STATUS_IDDLE;
    status_sub_algoritma = STATUS_IDDLE;
    status_sub_sub_algoritma = STATUS_IDDLE;
}

void algorithm_routine()
{
    switch (status_algoritma)
    {
    case STATUS_IDDLE:
        iddle_routine();

        if (status_sub_algoritma == 0)
            if (joy.right && !prev_joy.right)
            {
                _log.info("Standby STATUS_ROUTE_RECORD");
                status_algoritma = STATUS_ROUTE_RECORD;
            }
            else if (joy.left && !prev_joy.left)
            {
                _log.info("Standby STATUS_ROUTE_LOAD");
                status_algoritma = STATUS_ROUTE_LOAD;
            }
        break;

    case STATUS_ROUTE_RECORD:
        record_route_routine();

        if (status_sub_algoritma == 0)
            if (joy.right && !prev_joy.right)
            {
                _log.info("Standby STATUS_ROUTE_LOAD");
                status_algoritma = STATUS_ROUTE_LOAD;
            }
            else if (joy.left && !prev_joy.left)
            {
                _log.info("Standby STATUS_IDDLE");
                status_algoritma = STATUS_IDDLE;
            }
        break;

    case STATUS_ROUTE_LOAD:
        load_route_routine();

        if (status_sub_algoritma == 0)
            if (joy.right && !prev_joy.right)
            {
                _log.info("Standby STATUS_IDDLE");
                status_algoritma = STATUS_IDDLE;
            }
            else if (joy.left && !prev_joy.left)
            {
                _log.info("Standby STATUS_ROUTE_RECORD");
                status_algoritma = STATUS_ROUTE_RECORD;
            }
        break;

    default:
        _log.warn("Unknown State %d! Revert to Iddle.", status_algoritma);
        algorithm_reset();
        break;
    }
}

//------------------------------------------------------------------------------
//==============================================================================

void iddle_routine()
{
    _led.led(0.2, 0.2, 0.2, 0.2, 0.2);
}

//------------------------------------------------------------------------------
//==============================================================================

void record_route_routine()
{
    switch (status_sub_algoritma)
    {
    case STATUS_ROUTE_RECORD_IDDLE:
    {
        _led.led(0.2, 0.2, 0.2, 1.0, 1.0);
        if (joy.start && !prev_joy.start)
        {
            _log.info("Route Record - Started");
            status_sub_algoritma = STATUS_ROUTE_RECORD_STARTED;
        }
        break;
    }

    case STATUS_ROUTE_RECORD_STARTED:
    {
        // Waktu terakhir dan waktu sekarang
        static ros::Time time_last;
        static ros::Time time_now;

        // x dan y terakhir
        static double prev_x;
        static double prev_y;

        time_now = ros::Time::now();

        // Jika waktu terakhir state ini lebih dari 0.1 detik,
        // maka akan membuka file csv baru dan mencatat koordinat pertama
        if (time_now - time_last > ros::Duration(0.1))
        {
            prev_x = x;
            prev_y = y;

            //==========================

            time_t _time = time(NULL);
            struct tm _tm = *localtime(&_time);

            char buffer[64];
            strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &_tm);

            route_file.close();
            route_file.open(route_path + "/" + buffer + ".csv", std::ofstream::out | std::ofstream::app);

            //==========================

            route_file << "x_front,y_front,th_front,x_rear,y_rear,th_rear" << std::endl;
            route_file << x_front << "," << y_front << "," << th_front << "," << x_rear << "," << y_rear << "," << th_rear << std::endl;
        }

        // Mencatat waktu terakhir state ini
        time_last = time_now;

        // dx dan dy
        double dx = x - prev_x;
        double dy = y - prev_y;

        // Jika jarak antara koordinat terakhir dan koordinat saat ini
        // lebih dari 0.01, maka akan mencatat koordinat tersebut
        if (sqrt(dx * dx + dy * dy) >= 0.01)
        {
            prev_x = x;
            prev_y = y;

            //==========================

            route_file << x_front << "," << y_front << "," << th_front << "," << x_rear << "," << y_rear << "," << th_rear << std::endl;
        }

        _led.led(0.0, 0.2, 0.0, 0, 0);
        if (joy.start && !prev_joy.start)
        {
            _log.info("Route Record - Stopped");
            status_sub_algoritma = STATUS_ROUTE_RECORD_STOPPED;
        }
        break;
    }

    case STATUS_ROUTE_RECORD_STOPPED:
    {
        _led.led(0.2, 0.0, 0.0, 0, 0);
        if (joy.start && !prev_joy.start)
        {
            _log.info("Standby STATUS_ROUTE_RECORD");
            status_sub_algoritma = STATUS_ROUTE_RECORD_IDDLE;
        }
        break;
    }

    default:
    {
        _log.warn("Unknown State %d! Revert to Iddle.", status_sub_algoritma);
        algorithm_reset();
        break;
    }
    }
}

//------------------------------------------------------------------------------
//==============================================================================

void load_route_routine()
{
    static std::vector<std::string> route_name;
    static std::vector<std::string> route_size;
    static int8_t route_index = 0;

    switch (status_sub_algoritma)
    {
    case STATUS_ROUTE_LOAD_IDDLE:
    {
        _led.led(0.2, 0.2, 0.2, 2.0, 2.0);
        if (joy.start && !prev_joy.start)
        {
            _log.info("Route Load - List");
            status_sub_algoritma = STATUS_ROUTE_LOAD_LIST;
        }
        break;
    }

    case STATUS_ROUTE_LOAD_LIST:
    {
        route_name.clear();
        route_size.clear();

        for (const auto &entry : boost::filesystem::directory_iterator(route_path))
        {
            std::string file_name = entry.path().filename().string();
            std::string file_size = std::to_string(boost::filesystem::file_size(entry.path()));

            route_name.push_back(file_name);
            route_size.push_back(file_size);
        }

        if (route_name.size() != 0)
            _log.warn("%d Route Found", route_name.size());
        else
            _log.warn("No Route Found");

        for (int i = 0; i < route_name.size(); i++)
            _log.info("%d | %s | %s bytes", i, route_name[i].c_str(), route_size[i].c_str());

        if (route_name.size() != 0)
        {
            _log.info("Route 0 Selected");

            _log.info("Route Load - Select");
            status_sub_algoritma = STATUS_ROUTE_LOAD_SELECT;
        }
        else
        {
            _log.info("Standby STATUS_ROUTE_LOAD");
            status_sub_algoritma = STATUS_ROUTE_LOAD_IDDLE;
        }
        break;
    }

    case STATUS_ROUTE_LOAD_SELECT:
    {
        if (!prev_joy.up && joy.up)
        {
            if (++route_index >= route_name.size())
                route_index = 0;
            _log.info("Route %d Selected", route_index);
        }
        if (!prev_joy.down && joy.down)
        {
            if (--route_index < 0)
                route_index = route_name.size() - 1;
            _log.info("Route %d Selected", route_index);
        }

        _led.led(0.2, 0.2, 0.0, 0, 0);
        if (joy.start && !prev_joy.start)
        {
            _log.info("Route Load - Processed");
            status_sub_algoritma = STATUS_ROUTE_LOAD_PROCESSED;
        }
        break;
    }

    case STATUS_ROUTE_LOAD_PROCESSED:
    {
        marker_route_front.clear();
        marker_route_rear.clear();

        std::vector<std::string> row;
        std::string line, word;

        route_file.close();
        route_file.open(route_path + "/" + route_name[route_index], std::ofstream::in);

        // Abaikan baris pertama
        std::getline(route_file, line);

        // Baca baris selanjutnya
        while (std::getline(route_file, line))
        {
            row.clear();

            std::stringstream ss(line);

            while (std::getline(ss, word, ','))
                row.push_back(word);

            //==========================

            geometry_msgs::Point p;

            p.x = std::stod(row[0]);
            p.y = std::stod(row[1]);
            marker_route_front.push_back(p);

            p.x = std::stod(row[3]);
            p.y = std::stod(row[4]);
            marker_route_rear.push_back(p);
        }

        _log.info("Standby STATUS_ROUTE_LOAD");
        status_sub_algoritma = STATUS_ROUTE_LOAD_IDDLE;
        break;
    }

    default:
    {
        _log.warn("Unknown State %d! Revert to Iddle.", status_sub_algoritma);
        algorithm_reset();
        break;
    }
    }
}
