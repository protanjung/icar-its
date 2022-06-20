#ifndef HELP_MARKER_H_
#define HELP_MARKER_H_

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

class help_marker
{
private:
    //=====Publisher
    ros::Publisher pub_marker;

public:
    help_marker()
    {
    }

    ~help_marker()
    {
    }

    //==================================

    void init(ros::NodeHandle *NH)
    {
        pub_marker = NH->advertise<visualization_msgs::Marker>("marker", 0);
    }

    //==================================

    void add_line_strip(std::vector<geometry_msgs::Point> points, std::string frame_id, std::string ns, int32_t id, float r, float g, float b, double scale)
    {
        visualization_msgs::Marker msg_marker;

        msg_marker.header.frame_id = frame_id;
        msg_marker.header.stamp = ros::Time::now();

        msg_marker.ns = ns;
        msg_marker.id = id;

        msg_marker.type = visualization_msgs::Marker::LINE_STRIP;
        msg_marker.action = visualization_msgs::Marker::ADD;

        msg_marker.pose.orientation.w = 1.0;

        msg_marker.color.r = r;
        msg_marker.color.g = g;
        msg_marker.color.b = b;
        msg_marker.color.a = 1.0;

        msg_marker.scale.x = scale;

        for (int i = 0; i < points.size(); i++)
            msg_marker.points.push_back(points[i]);

        pub_marker.publish(msg_marker);
    }
};

#endif