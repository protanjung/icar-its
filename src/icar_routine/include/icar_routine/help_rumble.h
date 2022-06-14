#ifndef HELP_RUMBLE_H_
#define HELP_RUMBLE_H_

#include "icar_routine/ds4_from_pc_rumble.h"
#include "ros/ros.h"

class help_rumble
{
private:
    //=====Timer
    ros::Timer tim_25hz;
    //=====Publisher
    ros::Publisher pub_ds4_from_pc_rumble;

    float _small_strength;
    float _large_strength;

    //==================================

    void cllbck_tim_25hz(const ros::TimerEvent &event)
    {
        icar_routine::ds4_from_pc_rumble msg_ds4_from_pc_rumble;
        msg_ds4_from_pc_rumble.small_strength = _small_strength;
        msg_ds4_from_pc_rumble.large_strength = _large_strength;
        pub_ds4_from_pc_rumble.publish(msg_ds4_from_pc_rumble);
    }

public:
    help_rumble()
    {
    }

    ~help_rumble()
    {
    }

    //==================================

    void init(ros::NodeHandle *NH)
    {
        tim_25hz = NH->createTimer(ros::Duration(0.04), &help_rumble::cllbck_tim_25hz, this);

        pub_ds4_from_pc_rumble = NH->advertise<icar_routine::ds4_from_pc_rumble>("ds4_from_pc/rumble", 1);
    }

    //==================================

    void rumble(float small_strength, float large_strength)
    {
        _small_strength = small_strength;
        _large_strength = large_strength;
    }
};

#endif