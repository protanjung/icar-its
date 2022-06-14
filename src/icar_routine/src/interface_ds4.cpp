#include "ds4_driver/Feedback.h"
#include "ds4_driver/Status.h"
#include "icar_routine/ds4_from_pc_led.h"
#include "icar_routine/ds4_from_pc_rumble.h"
#include "icar_routine/ds4_to_pc_joy.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_tim_25hz(const ros::TimerEvent &event);

void cllbck_sub_ds4_raw_to_pc(const ds4_driver::StatusConstPtr &msg);
void cllbck_sub_ds4_from_pc_led(const icar_routine::ds4_from_pc_ledConstPtr &msg);
void cllbck_sub_ds4_from_pc_rumble(const icar_routine::ds4_from_pc_rumbleConstPtr &msg);

//=====Timer
ros::Timer tim_25hz;
//=====Subscriber
ros::Subscriber sub_ds4_raw_to_pc;
ros::Subscriber sub_ds4_from_pc_led;
ros::Subscriber sub_ds4_from_pc_rumble;
//=====Publisher
ros::Publisher pub_ds4_raw_from_pc;
ros::Publisher pub_ds4_to_pc_joy;

//-----DS4 from PC
ds4_driver::Feedback ds4_raw_from_pc;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface_ds4");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_25hz = NH.createTimer(ros::Duration(0.08), cllbck_tim_25hz);
    //=====Subscriber
    sub_ds4_raw_to_pc = NH.subscribe("ds4_raw/to_pc", 1, cllbck_sub_ds4_raw_to_pc);
    sub_ds4_from_pc_led = NH.subscribe("ds4_from_pc/led", 1, cllbck_sub_ds4_from_pc_led);
    sub_ds4_from_pc_rumble = NH.subscribe("ds4_from_pc/rumble", 1, cllbck_sub_ds4_from_pc_rumble);
    //=====Publisher
    pub_ds4_raw_from_pc = NH.advertise<ds4_driver::Feedback>("ds4_raw/from_pc", 1);
    pub_ds4_to_pc_joy = NH.advertise<icar_routine::ds4_to_pc_joy>("ds4_to_pc/joy", 1);

    AS.start();
    ros::waitForShutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_25hz(const ros::TimerEvent &event)
{
    ds4_driver::Feedback msg_ds4_raw_from_pc = ds4_raw_from_pc;
    pub_ds4_raw_from_pc.publish(msg_ds4_raw_from_pc);
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_ds4_raw_to_pc(const ds4_driver::StatusConstPtr &msg)
{
    icar_routine::ds4_to_pc_joy msg_ds4_to_pc_joy;
    msg_ds4_to_pc_joy.up = msg->button_dpad_up;
    msg_ds4_to_pc_joy.left = msg->button_dpad_left;
    msg_ds4_to_pc_joy.down = msg->button_dpad_down;
    msg_ds4_to_pc_joy.right = msg->button_dpad_right;
    msg_ds4_to_pc_joy.triangle = msg->button_triangle;
    msg_ds4_to_pc_joy.square = msg->button_square;
    msg_ds4_to_pc_joy.cross = msg->button_cross;
    msg_ds4_to_pc_joy.circle = msg->button_circle;
    msg_ds4_to_pc_joy.select = msg->button_share;
    msg_ds4_to_pc_joy.start = msg->button_options;
    msg_ds4_to_pc_joy.ps = msg->button_ps;
    msg_ds4_to_pc_joy.trackpad = msg->button_trackpad;
    msg_ds4_to_pc_joy.l1 = msg->button_l1;
    msg_ds4_to_pc_joy.l2 = true ? msg->axis_l2 == 1 : false;
    msg_ds4_to_pc_joy.l3 = msg->button_l3;
    msg_ds4_to_pc_joy.r1 = msg->button_r1;
    msg_ds4_to_pc_joy.r2 = true ? msg->axis_r2 == 1 : false;
    msg_ds4_to_pc_joy.r3 = msg->button_r3;
    msg_ds4_to_pc_joy.analog_l2 = msg->axis_l2;
    msg_ds4_to_pc_joy.analog_r2 = msg->axis_r2;
    msg_ds4_to_pc_joy.analog_x1 = -msg->axis_left_x;
    msg_ds4_to_pc_joy.analog_y1 = msg->axis_left_y;
    msg_ds4_to_pc_joy.analog_x2 = -msg->axis_right_x;
    msg_ds4_to_pc_joy.analog_y2 = msg->axis_right_y;
    pub_ds4_to_pc_joy.publish(msg_ds4_to_pc_joy);
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_ds4_from_pc_led(const icar_routine::ds4_from_pc_ledConstPtr &msg)
{
    static icar_routine::ds4_from_pc_led prev_msg;

    if (prev_msg.color.size() == 0)
        prev_msg.color.resize(3);

    if (msg->color[0] != prev_msg.color[0] ||
        msg->color[1] != prev_msg.color[1] ||
        msg->color[2] != prev_msg.color[2])
    {
        ds4_raw_from_pc.set_led = 1;
        ds4_raw_from_pc.led_r = msg->color[0];
        ds4_raw_from_pc.led_g = msg->color[1];
        ds4_raw_from_pc.led_b = msg->color[2];
    }

    if (msg->blink_on_duration != prev_msg.blink_on_duration ||
        msg->blink_off_duration != prev_msg.blink_off_duration)
    {
        ds4_raw_from_pc.set_led_flash = 1;
        ds4_raw_from_pc.led_flash_on = msg->blink_on_duration;
        ds4_raw_from_pc.led_flash_off = msg->blink_off_duration;
    }

    prev_msg = *msg;
}

void cllbck_sub_ds4_from_pc_rumble(const icar_routine::ds4_from_pc_rumbleConstPtr &msg)
{
    ds4_raw_from_pc.set_rumble = 1;
    ds4_raw_from_pc.rumble_big = msg->large_strength;
    ds4_raw_from_pc.rumble_small = msg->small_strength;
}