#include "arpa/inet.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16MultiArray.h"

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_stm32_from_pc_throttle(const std_msgs::Int16ConstPtr &msg);
void cllbck_sub_stm32_from_pc_steering(const std_msgs::Int16ConstPtr &msg);
void cllbck_sub_stm32_from_pc_transmission(const std_msgs::Int8ConstPtr &msg);

int stm32_init();
int stm32_routine();

void remote_routine();
void rotary_encoder_routine();
void gyroscope_routine();
void throttle_position_routine();
void steering_position_routine();

//=====Parameter
std::string stm32_ip;
int stm32_port;
//=====Timer
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_stm32_from_pc_throttle;
ros::Subscriber sub_stm32_from_pc_steering;
ros::Subscriber sub_stm32_from_pc_transmission;
//=====Publisher
ros::Publisher pub_stm32_to_pc_remote;
ros::Publisher pub_stm32_to_pc_rotary_encoder;
ros::Publisher pub_stm32_to_pc_gyroscope;
ros::Publisher pub_stm32_to_pc_throttle_position;
ros::Publisher pub_stm32_to_pc_steering_position;

//-----UDP Connection
//===================
int sockfd = 0;
struct sockaddr_in server_address;
uint8_t buffer_tx[1024] = {0};
uint8_t buffer_rx[1024] = {0};

//-----STM32 to PC
//================
uint32_t epoch_to_pc;
uint16_t remote[16];
uint16_t encoder_kiri;
uint16_t encoder_kanan;
float gyroscope;
int16_t throttle_position;
int16_t steering_position;

//-----STM32 fom PC
//=================
uint32_t epoch_from_pc;
int16_t throttle;
int16_t steering;
int8_t transmission;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface_stm32");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("stm32/ip", stm32_ip);
    NH.getParam("stm32/port", stm32_port);
    //=====Timer
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_stm32_from_pc_throttle = NH.subscribe("stm32_from_pc/throttle", 1, cllbck_sub_stm32_from_pc_throttle);
    sub_stm32_from_pc_steering = NH.subscribe("stm32_from_pc/steering", 1, cllbck_sub_stm32_from_pc_steering);
    sub_stm32_from_pc_transmission = NH.subscribe("stm32_from_pc/transmission", 1, cllbck_sub_stm32_from_pc_transmission);
    //=====Publisher
    pub_stm32_to_pc_remote = NH.advertise<std_msgs::UInt16MultiArray>("stm32_to_pc/remote", 1);
    pub_stm32_to_pc_rotary_encoder = NH.advertise<std_msgs::UInt16MultiArray>("stm32_to_pc/rotary_encoder", 1);
    pub_stm32_to_pc_gyroscope = NH.advertise<std_msgs::Float32>("stm32_to_pc/gyroscope", 1);
    pub_stm32_to_pc_throttle_position = NH.advertise<std_msgs::Int16>("stm32_to_pc/throttle_position", 1);
    pub_stm32_to_pc_steering_position = NH.advertise<std_msgs::Int16>("stm32_to_pc/steering_position", 1);

    if (stm32_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
    if (stm32_routine() == -1)
        ros::shutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_stm32_from_pc_throttle(const std_msgs::Int16ConstPtr &msg)
{
    throttle = msg->data;
}

void cllbck_sub_stm32_from_pc_steering(const std_msgs::Int16ConstPtr &msg)
{
    steering = msg->data;
}

void cllbck_sub_stm32_from_pc_transmission(const std_msgs::Int8ConstPtr &msg)
{
    transmission = msg->data;
}

//------------------------------------------------------------------------------
//==============================================================================

int stm32_init()
{
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0)
        return -1;

    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(stm32_port);
    server_address.sin_addr.s_addr = inet_addr(stm32_ip.c_str());

    return 0;
}

int stm32_routine()
{
    memcpy(buffer_tx + 0, &epoch_from_pc, 4);
    memcpy(buffer_tx + 4, &throttle, 2);
    memcpy(buffer_tx + 6, &steering, 2);
    memcpy(buffer_tx + 8, &transmission, 1);

    int len_stm32_from_pc = sendto(sockfd, buffer_tx, 11, MSG_DONTWAIT, (struct sockaddr *)&server_address, sizeof(server_address));
    int len_stm32_to_pc = recvfrom(sockfd, buffer_rx, 50, MSG_DONTWAIT, NULL, NULL);

    memcpy(&epoch_to_pc, buffer_rx + 0, 4);
    for (int i = 0; i < 16; i++)
        memcpy(&remote[i], buffer_rx + 4 + i * 2, 2);
    memcpy(&encoder_kiri, buffer_rx + 36, 2);
    memcpy(&encoder_kanan, buffer_rx + 38, 2);
    memcpy(&gyroscope, buffer_rx + 40, 4);
    memcpy(&throttle_position, buffer_rx + 44, 2);
    memcpy(&steering_position, buffer_rx + 46, 2);

    if (len_stm32_from_pc == -1 || len_stm32_to_pc == -1)
        return 0;

    remote_routine();
    rotary_encoder_routine();
    gyroscope_routine();
    throttle_position_routine();
    steering_position_routine();

    epoch_from_pc++;

    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

void remote_routine()
{
    std_msgs::UInt16MultiArray msg_stm32_to_pc_remote;
    for (int i = 0; i < 16; i++)
        msg_stm32_to_pc_remote.data.push_back(remote[i]);
    pub_stm32_to_pc_remote.publish(msg_stm32_to_pc_remote);
}

void rotary_encoder_routine()
{
    std_msgs::UInt16MultiArray msg_stm32_to_pc_rotary_encoder;
    msg_stm32_to_pc_rotary_encoder.data.push_back(encoder_kiri);
    msg_stm32_to_pc_rotary_encoder.data.push_back(encoder_kanan);
    pub_stm32_to_pc_rotary_encoder.publish(msg_stm32_to_pc_rotary_encoder);
}

void gyroscope_routine()
{
    std_msgs::Float32 msg_stm32_to_pc_gyroscope;
    msg_stm32_to_pc_gyroscope.data = gyroscope;
    pub_stm32_to_pc_gyroscope.publish(msg_stm32_to_pc_gyroscope);
}

void throttle_position_routine()
{
    std_msgs::Int16 msg_stm32_to_pc_throttle_position;
    msg_stm32_to_pc_throttle_position.data = throttle_position;
    pub_stm32_to_pc_throttle_position.publish(msg_stm32_to_pc_throttle_position);
}

void steering_position_routine()
{
    std_msgs::Int16 msg_stm32_to_pc_steering_position;
    msg_stm32_to_pc_steering_position.data = steering_position;
    pub_stm32_to_pc_steering_position.publish(msg_stm32_to_pc_steering_position);
}
