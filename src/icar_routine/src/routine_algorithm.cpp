#include "icar_routine/routine.h"

void algorithm_routine()
{
    static int16_t throttle_now = 0;
    static int16_t steering_now = 0;
    static int8_t transmission_now = 0;

    switch (status_algoritma)
    {
    case STATUS_IDDLE:
        _led.led(0.5, 0.5, 0.5, 0.2, 0.8);

        jalan_manual(0, 0, 0);

        if (!prev_joy.ps && joy.ps)
        {
            transmission_now = 1;
            status_algoritma = STATUS_TEST;
        }

        break;

    case STATUS_TEST:

        // Hitung throttle
        if (joy.analog_l2 > 0)
            throttle_now = joy.analog_l2 * -100;
        else
            throttle_now = joy.analog_r2 * 100;

        // Hitung steering
        steering_now = joy.analog_x1 * 5000;

        // Hitung transmission
        if (!prev_joy.r1 && joy.r1)
            if (transmission_now == 1)
            {
                transmission_now = -1;
                _led.led(1.0, 0.0, 0.5, 0.2, 0.8);
            }
            else if (transmission_now == -1)
            {
                transmission_now = 1;
                _led.led(0.0, 1.0, 0.5, 0.2, 0.8);
            }

        jalan_manual(throttle_now, steering_now, transmission_now);

        if (!prev_joy.ps && joy.ps)
        {
            transmission_now = 0;
            status_algoritma = STATUS_IDDLE;
        }

        break;
    }
}