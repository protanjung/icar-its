#include "icar_routine/routine.h"

void jalan_manual(float _throttle, float _steering, uint8_t _transmission)
{
    static float steering_buffer = 0;

    steering_buffer = steering_buffer * 0.87 + _steering * 0.13;

    std_msgs::Int16 msg_throttle;
    msg_throttle.data = _throttle;
    pub_throttle.publish(msg_throttle);

    std_msgs::Int16 msg_steering;
    msg_steering.data = steering_buffer;
    pub_steering.publish(msg_steering);

    std_msgs::Int8 msg_transmission;
    msg_transmission.data = _transmission;
    pub_transmission.publish(msg_transmission);
}