#ifndef HELP_JOY_H_
#define HELP_JOY_H_

#include "icar_routine/ds4_to_pc_joy.h"
#include "ros/ros.h"

class help_joy
{
private:
    //=====Subscriber
    ros::Subscriber sub_ds4_to_pc_joy;

    icar_routine::ds4_to_pc_joy _joy;

    //==================================

    void cllbck_sub_ds4_to_pc_joy(const icar_routine::ds4_to_pc_joyConstPtr &msg)
    {
        _joy = *msg;
    }

public:
    help_joy()
    {
    }

    ~help_joy()
    {
    }

    //==================================

    void init(ros::NodeHandle *NH)
    {
        sub_ds4_to_pc_joy = NH->subscribe("ds4_to_pc/joy", 1, &help_joy::cllbck_sub_ds4_to_pc_joy, this);
    }

    //==================================

    void joy(icar_routine::ds4_to_pc_joy &joy, icar_routine::ds4_to_pc_joy &prev_joy)
    {

        prev_joy = joy;
        joy = _joy;
    }
};

#endif