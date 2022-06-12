#include "ds4_driver/Feedback.h"
#include "ds4_driver/Status.h"
#include "icar_hardware/ds4_led.h"
#include "icar_hardware/ds4_rumble.h"
#include "icar_hardware/stm32_from_pc.h"
#include "icar_hardware/stm32_to_pc.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt16MultiArray.h"

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_stm32_to_pc(const icar_hardware::stm32_to_pcConstPtr &msg);
void cllbck_sub_ds4_to_pc(const ds4_driver::Status &msg);
void cllbck_sub_ds4_led(const icar_hardware::ds4_ledConstPtr &msg);
void cllbck_sub_ds4_rumble(const icar_hardware::ds4_rumbleConstPtr &msg);

void stm32_to_pc_routine();
void stm32_from_pc_routine();
void ds4_to_pc_routine();
void ds4_from_pc_routine();

//=====Timer
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_stm32_to_pc;
ros::Subscriber sub_ds4_to_pc;
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

// STM32 from PC
int16_t throttle;
int16_t steering;
int8_t transmission;

// STM32 to PC
uint16_t remote[16];

uint16_t encoder_kiri;
uint16_t encoder_kanan;
float gyroscope;

int16_t throttle_position;
int16_t steering_position;

// DS4 from PC
bool led_new_command;
bool rumble_new_command;
uint8_t led_status, prev_led_status;
uint8_t rumble_status, prev_rumble_status;

float led_color[3];
float led_duration;
float led_value;

float rumble_duration;
float rumble_value;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_interpreter");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_stm32_to_pc = NH.subscribe("/stm32/to_pc", 1, cllbck_sub_stm32_to_pc);
    sub_ds4_to_pc = NH.subscribe("/ds4/to_pc", 1, cllbck_sub_ds4_to_pc);
    sub_ds4_led = NH.subscribe("/ds4_/led", 1, cllbck_sub_ds4_led);
    sub_ds4_rumble = NH.subscribe("/ds4_/rumble", 1, cllbck_sub_ds4_rumble);
    //=====Publisher
    pub_stm32_from_pc = NH.advertise<icar_hardware::stm32_from_pc>("/stm32/from_pc", 1);
    pub_ds4_from_pc = NH.advertise<ds4_driver::Feedback>("/ds4/from_pc", 1);
    pub_stm32_remote = NH.advertise<std_msgs::UInt16MultiArray>("/stm32_/remote", 1);
    pub_stm32_rotary_encoder = NH.advertise<std_msgs::UInt16MultiArray>("/stm32_/rotary_encoder", 1);
    pub_stm32_gyroscope = NH.advertise<std_msgs::Float32>("/stm32_/gyroscope", 1);
    pub_stm32_throttle_position = NH.advertise<std_msgs::Int16>("/stm32_/throttle_position", 1);
    pub_stm32_steering_position = NH.advertise<std_msgs::Int16>("/stm32_/steering_position", 1);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
    ds4_from_pc_routine();
    stm32_to_pc_routine();
    stm32_from_pc_routine();
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

void stm32_from_pc_routine()
{
    icar_hardware::stm32_from_pc msg_stm32_from_pc;
    msg_stm32_from_pc.throttle = throttle;
    msg_stm32_from_pc.steering = steering;
    msg_stm32_from_pc.transmission = transmission;
    pub_stm32_from_pc.publish(msg_stm32_from_pc);
}

void ds4_to_pc_routine()
{
}

void ds4_from_pc_routine()
{
    static ros::Time time_now;
    static ros::Time time_prev;

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
