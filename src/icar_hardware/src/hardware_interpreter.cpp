#include "ds4_driver/Feedback.h"
#include "ds4_driver/Status.h"
#include "icar_hardware/ds4_battery.h"
#include "icar_hardware/ds4_joy.h"
#include "icar_hardware/ds4_led.h"
#include "icar_hardware/ds4_rumble.h"
#include "icar_hardware/stm32_from_pc.h"
#include "icar_hardware/stm32_to_pc.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt16MultiArray.h"

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_stm32_to_pc(const icar_hardware::stm32_to_pcConstPtr &msg);
void cllbck_sub_ds4_to_pc(const ds4_driver::Status &msg);
void cllbck_sub_stm32_throttle(const std_msgs::Int16ConstPtr &msg);
void cllbck_sub_stm32_steering(const std_msgs::Int16ConstPtr &msg);
void cllbck_sub_stm32_transmission(const std_msgs::Int8ConstPtr &msg);
void cllbck_sub_ds4_led(const icar_hardware::ds4_ledConstPtr &msg);
void cllbck_sub_ds4_rumble(const icar_hardware::ds4_rumbleConstPtr &msg);

void stm32_from_pc_routine();
void stm32_to_pc_routine();
void ds4_from_pc_routine();
void ds4_to_pc_routine();

//=====Timer
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_stm32_to_pc;
ros::Subscriber sub_ds4_to_pc;
ros::Subscriber sub_stm32_throttle;
ros::Subscriber sub_stm32_steering;
ros::Subscriber sub_stm32_transmission;
ros::Subscriber sub_ds4_led;
ros::Subscriber sub_ds4_rumble;
//=====Publisher
ros::Publisher pub_stm32_from_pc;
ros::Publisher pub_ds4_from_pc;
ros::Publisher pub_stm32_remote;
ros::Publisher pub_stm32_rotary_encoder;
ros::Publisher pub_stm32_gyroscope;
ros::Publisher pub_stm32_throttle_position;
ros::Publisher pub_stm32_steering_position;
ros::Publisher pub_ds4_joy;
ros::Publisher pub_ds4_battery;

//==============
// STM32 from PC
//==============
int16_t throttle;
int16_t steering;
int8_t transmission;

//============
// STM32 to PC
//============
uint16_t remote[16];

uint16_t encoder_kiri;
uint16_t encoder_kanan;
float gyroscope;

int16_t throttle_position;
int16_t steering_position;

//============
// DS4 from PC
//============
bool led_new_command;
bool rumble_new_command;

float led_color[3];
float led_duration;
float led_value;

float rumble_duration;
float rumble_value;

//==========
// DS4 to PC
//==========
icar_hardware::ds4_joy ds4_joy;
icar_hardware::ds4_battery ds4_battery;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_interpreter");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);

    //=====Subscriber
    sub_stm32_to_pc = NH.subscribe("/stm32_/to_pc", 1, cllbck_sub_stm32_to_pc);
    sub_ds4_to_pc = NH.subscribe("/ds4_/to_pc", 1, cllbck_sub_ds4_to_pc);

    sub_stm32_throttle = NH.subscribe("/stm32/throttle", 1, cllbck_sub_stm32_throttle);
    sub_stm32_steering = NH.subscribe("/stm32/steering", 1, cllbck_sub_stm32_steering);
    sub_stm32_transmission = NH.subscribe("/stm32/transmission", 1, cllbck_sub_stm32_transmission);

    sub_ds4_led = NH.subscribe("/ds4/led", 1, cllbck_sub_ds4_led);
    sub_ds4_rumble = NH.subscribe("/ds4/rumble", 1, cllbck_sub_ds4_rumble);

    //=====Publisher
    pub_stm32_from_pc = NH.advertise<icar_hardware::stm32_from_pc>("/stm32_/from_pc", 1);
    pub_ds4_from_pc = NH.advertise<ds4_driver::Feedback>("/ds4_/from_pc", 1);

    pub_stm32_remote = NH.advertise<std_msgs::UInt16MultiArray>("/stm32/remote", 1);
    pub_stm32_rotary_encoder = NH.advertise<std_msgs::UInt16MultiArray>("/stm32/rotary_encoder", 1);
    pub_stm32_gyroscope = NH.advertise<std_msgs::Float32>("/stm32/gyroscope", 1);
    pub_stm32_throttle_position = NH.advertise<std_msgs::Int16>("/stm32/throttle_position", 1);
    pub_stm32_steering_position = NH.advertise<std_msgs::Int16>("/stm32/steering_position", 1);

    pub_ds4_joy = NH.advertise<icar_hardware::ds4_joy>("/ds4/joy", 1);
    pub_ds4_battery = NH.advertise<icar_hardware::ds4_battery>("/ds4/battery", 1);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
    ds4_from_pc_routine();
    ds4_to_pc_routine();
    stm32_from_pc_routine();
    stm32_to_pc_routine();
}

//==============================================================================

void cllbck_sub_stm32_to_pc(const icar_hardware::stm32_to_pcConstPtr &msg)
{
    for (int i = 0; i < 16; i++)
        remote[i] = msg->remote[i];

    encoder_kiri = msg->encoder_kiri;
    encoder_kanan = msg->encoder_kanan;
    gyroscope = msg->gyroscope;

    throttle_position = msg->throttle_position;
    steering_position = msg->steering_position;
}

void cllbck_sub_ds4_to_pc(const ds4_driver::Status &msg)
{
    ds4_joy.down = msg.button_dpad_down;
    ds4_joy.right = msg.button_dpad_right;
    ds4_joy.up = msg.button_dpad_up;
    ds4_joy.left = msg.button_dpad_left;

    ds4_joy.cross = msg.button_cross;
    ds4_joy.circle = msg.button_circle;
    ds4_joy.triangle = msg.button_triangle;
    ds4_joy.square = msg.button_square;

    ds4_joy.l1 = msg.button_l1;
    ds4_joy.l2 = msg.button_l2;
    ds4_joy.l3 = msg.button_l3;
    ds4_joy.l2_analog = msg.axis_l2;
    ds4_joy.r1 = msg.button_r1;
    ds4_joy.r2 = msg.button_r2;
    ds4_joy.r3 = msg.button_r3;
    ds4_joy.r2_analog = msg.axis_r2;

    ds4_joy.select = msg.button_share;
    ds4_joy.start = msg.button_options;
    ds4_joy.ps = msg.button_ps;
    ds4_joy.trackpad = msg.button_trackpad;

    ds4_joy.left_x = msg.axis_left_x;
    ds4_joy.left_y = msg.axis_left_y;
    ds4_joy.right_x = msg.axis_right_x;
    ds4_joy.right_y = msg.axis_right_y;

    ds4_joy.trackpad0_status = msg.touch0.active;
    ds4_joy.trackpad0_x = msg.touch0.x;
    ds4_joy.trackpad0_y = msg.touch0.y;
    ds4_joy.trackpad1_status = msg.touch1.active;
    ds4_joy.trackpad1_x = msg.touch1.x;
    ds4_joy.trackpad1_y = msg.touch1.y;

    ds4_battery.percentage = msg.battery_percentage;
}

//==============================================================================

void cllbck_sub_stm32_throttle(const std_msgs::Int16ConstPtr &msg)
{
    throttle = msg->data;
}

void cllbck_sub_stm32_steering(const std_msgs::Int16ConstPtr &msg)
{
    steering = msg->data;
}

void cllbck_sub_stm32_transmission(const std_msgs::Int8ConstPtr &msg)
{
    transmission = msg->data;
}

//==============================================================================

void cllbck_sub_ds4_led(const icar_hardware::ds4_ledConstPtr &msg)
{
    led_new_command = true;

    for (int i = 0; i < 3; i++)
        led_color[i] = msg->led_color[i];
    led_duration = msg->led_duration;
    led_value = msg->led_value;
}

void cllbck_sub_ds4_rumble(const icar_hardware::ds4_rumbleConstPtr &msg)
{
    rumble_new_command = true;

    rumble_duration = msg->rumble_duration;
    rumble_value = msg->rumble_value;
}

//==============================================================================

void stm32_from_pc_routine()
{
    static int16_t throttle_now;
    static int16_t steering_now;

    if (throttle_now < throttle)
    {
        throttle_now += throttle;
        if (throttle_now > throttle)
            throttle_now = throttle;
    }
    else if (throttle_now > throttle)
    {
        throttle_now -= 1;
        if (throttle_now < throttle)
            throttle_now = throttle;
    }

    if (steering_now < steering)
    {
        steering_now += 1;
        if (steering_now > steering)
            steering_now = steering;
    }
    else if (steering_now > steering)
    {
        steering_now -= 1;
        if (steering_now < steering)
            steering_now = steering;
    }

    icar_hardware::stm32_from_pc msg_stm32_from_pc;
    msg_stm32_from_pc.throttle = throttle_now;
    msg_stm32_from_pc.steering = steering_now;
    msg_stm32_from_pc.transmission = transmission;
    pub_stm32_from_pc.publish(msg_stm32_from_pc);
}

void stm32_to_pc_routine()
{
    std_msgs::UInt16MultiArray msg_remote;
    for (int i = 0; i < 16; i++)
        msg_remote.data.push_back(remote[i]);
    pub_stm32_remote.publish(msg_remote);

    //==================================

    std_msgs::UInt16MultiArray msg_rotary_encoder;
    msg_rotary_encoder.data.push_back(encoder_kiri);
    msg_rotary_encoder.data.push_back(encoder_kanan);
    pub_stm32_rotary_encoder.publish(msg_rotary_encoder);

    //==================================

    std_msgs::Float32 msg_gyroscope;
    msg_gyroscope.data = gyroscope;
    pub_stm32_gyroscope.publish(msg_gyroscope);

    //==================================

    std_msgs::Int16 msg_throttle_position;
    msg_throttle_position.data = throttle_position;
    pub_stm32_throttle_position.publish(msg_throttle_position);

    //==================================

    std_msgs::Int16 msg_steering_position;
    msg_steering_position.data = steering_position;
    pub_stm32_steering_position.publish(msg_steering_position);
}

void ds4_from_pc_routine()
{
    static ros::Time time_now;
    static ros::Time time_prev;
    static uint8_t led_status, prev_led_status;
    static uint8_t rumble_status, prev_rumble_status;

    time_prev = time_now;
    time_now = ros::Time::now();

    if (led_duration > 0)
    {
        led_status = 1;
        led_duration -= time_now.toSec() - time_prev.toSec();
        if (led_duration <= 0)
        {
            led_status = 0;
            led_duration = 0;
        }
    }

    if (rumble_duration > 0)
    {
        rumble_status = 1;
        rumble_duration -= time_now.toSec() - time_prev.toSec();
        if (rumble_duration <= 0)
        {
            rumble_status = 0;
            rumble_duration = 0;
        }
    }

    if (led_new_command || rumble_new_command ||
        prev_led_status != led_status || prev_rumble_status != rumble_status)
    {
        led_new_command = false;
        rumble_new_command = false;

        ds4_driver::Feedback msg_ds4_from_pc;
        msg_ds4_from_pc.led_r = led_color[0];
        msg_ds4_from_pc.led_g = led_color[1];
        msg_ds4_from_pc.led_b = led_color[2];
        msg_ds4_from_pc.set_led = led_status;
        msg_ds4_from_pc.set_led_flash = 1;
        msg_ds4_from_pc.led_flash_on = led_value;
        msg_ds4_from_pc.led_flash_off = led_value ? led_status : 0;
        msg_ds4_from_pc.set_rumble = rumble_status;
        msg_ds4_from_pc.rumble_duration = rumble_duration;
        msg_ds4_from_pc.rumble_small = rumble_value;
        msg_ds4_from_pc.rumble_big = rumble_value;
        pub_ds4_from_pc.publish(msg_ds4_from_pc);
    }

    prev_led_status = led_status;
    prev_rumble_status = rumble_status;
}

void ds4_to_pc_routine()
{
    ds4_joy.left_x *= -1;
    ds4_joy.right_x *= -1;

    icar_hardware::ds4_joy msg_ds4_joy = ds4_joy;
    pub_ds4_from_pc.publish(msg_ds4_joy);

    icar_hardware::ds4_battery msg_ds4_battery = ds4_battery;
    pub_ds4_battery.publish(msg_ds4_battery);
}