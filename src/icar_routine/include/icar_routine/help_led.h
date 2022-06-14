#ifndef HELP_LED_H_
#define HELP_LED_H_

#include "icar_routine/ds4_from_pc_led.h"
#include "ros/ros.h"

class help_led
{
private:
    //=====Timer
    ros::Timer tim_25hz;
    //=====Publisher
    ros::Publisher pub_ds4_from_pc_led;

    float _red, _green, _blue;
    float _blink_on_duration;
    float _blink_off_duration;

    //==================================

    void cllbck_tim_25hz(const ros::TimerEvent &event)
    {
        icar_routine::ds4_from_pc_led msg_ds4_from_pc_led;
        msg_ds4_from_pc_led.color.push_back(_red);
        msg_ds4_from_pc_led.color.push_back(_green);
        msg_ds4_from_pc_led.color.push_back(_blue);
        msg_ds4_from_pc_led.blink_on_duration = _blink_on_duration;
        msg_ds4_from_pc_led.blink_off_duration = _blink_off_duration;
        pub_ds4_from_pc_led.publish(msg_ds4_from_pc_led);
    }

public:
    help_led()
    {
    }

    ~help_led()
    {
    }

    //==================================

    void init(ros::NodeHandle *NH)
    {
        tim_25hz = NH->createTimer(ros::Duration(0.04), &help_led::cllbck_tim_25hz, this);

        pub_ds4_from_pc_led = NH->advertise<icar_routine::ds4_from_pc_led>("ds4_from_pc/led", 1);
    }

    //==================================

    void led(float red, float green, float blue)
    {
        _red = red;
        _green = green;
        _blue = blue;
    }

    void led(float red, float green, float blue, float on_duration, float off_duration)
    {
        _red = red;
        _green = green;
        _blue = blue;
        _blink_on_duration = on_duration;
        _blink_off_duration = off_duration;
    }

    void led(float on_duration, float off_duration)
    {
        _blink_on_duration = on_duration;
        _blink_off_duration = off_duration;
    }
};

#endif