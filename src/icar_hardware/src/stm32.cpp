#include "arpa/inet.h"
#include "icar_hardware/stm32_from_pc.h"
#include "icar_hardware/stm32_to_pc.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_stm32_from_pc(const icar_hardware::stm32_from_pcConstPtr &msg);

int stm32_init();
int stm32_routine();

//=====Parameter
std::string stm32_ip;
int stm32_port;
//=====Timer
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_stm32_from_pc;
//=====Publisher
ros::Publisher pub_stm32_to_pc;

// UDP Connection
int sockfd = 0;
struct sockaddr_in server_address;
uint8_t buffer_tx[1024] = {0};
uint8_t buffer_rx[1024] = {0};

// STM32 from PC
uint32_t epoch_from_pc;
int16_t throttle;
int16_t steering;
int8_t transmission;

// STM32 to PC
uint32_t epoch_to_pc;
uint16_t remote[16];
uint16_t encoder_kiri;
uint16_t encoder_kanan;
float gyroscope;
int16_t throttle_position;
int16_t steering_position;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stm32");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Parameter
    NH.getParam("/stm32/ip", stm32_ip);
    NH.getParam("/stm32/port", stm32_port);
    //=====Timer
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_stm32_from_pc = NH.subscribe("/stm32_/from_pc", 1, cllbck_sub_stm32_from_pc);
    //=====Publisher
    pub_stm32_to_pc = NH.advertise<icar_hardware::stm32_to_pc>("/stm32_/to_pc", 1);

    if (stm32_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
    if (stm32_routine() == -1)
        ros::shutdown();
}

//==============================================================================

void cllbck_sub_stm32_from_pc(const icar_hardware::stm32_from_pcConstPtr &msg)
{
    throttle = msg->throttle;
    steering = msg->steering;
    transmission = msg->transmission;
}

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
    epoch_from_pc++;

    //==================================

    memcpy(buffer_tx + 0, &epoch_from_pc, 4);
    memcpy(buffer_tx + 4, &throttle, 2);
    memcpy(buffer_tx + 6, &steering, 2);
    memcpy(buffer_tx + 8, &transmission, 1);

    int data_len_from_pc = sendto(sockfd, buffer_tx, 11, MSG_DONTWAIT, (struct sockaddr *)&server_address, (socklen_t)sizeof(server_address));
    int data_len_to_pc = recvfrom(sockfd, buffer_rx, 50, MSG_DONTWAIT, NULL, NULL);

    memcpy(&epoch_to_pc, buffer_rx + 0, 4);
    for (int i = 0; i < 16; i++)
        memcpy(&remote[i], buffer_rx + 4 + i * 2, 2);
    memcpy(&encoder_kiri, buffer_rx + 36, 2);
    memcpy(&encoder_kanan, buffer_rx + 38, 2);
    memcpy(&gyroscope, buffer_rx + 40, 4);
    memcpy(&throttle_position, buffer_rx + 44, 2);
    memcpy(&steering_position, buffer_rx + 46, 2);

    //==================================

    if (data_len_from_pc == -1 || data_len_to_pc == -1)
        return 0;

    //==================================

    icar_hardware::stm32_to_pc msg_stm32_to_pc;
    for (int i = 0; i < 16; i++)
        msg_stm32_to_pc.remote.push_back(remote[i]);
    msg_stm32_to_pc.encoder_kiri = encoder_kiri;
    msg_stm32_to_pc.encoder_kanan = encoder_kanan;
    msg_stm32_to_pc.gyroscope = gyroscope;
    msg_stm32_to_pc.throttle_position = throttle_position;
    msg_stm32_to_pc.steering_position = steering_position;
    pub_stm32_to_pc.publish(msg_stm32_to_pc);

    return 0;
}