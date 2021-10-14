#ifndef TB67H420FTG_H
#define TB67H420FTG_H

#include "Arduino.h"

#define PWM_RESOLUTION_BITS 12
#define PWM_FREQUENCY_HZ 20000

typedef uint8_t PinName;

class TB67H420FTG{
    public:
        TB67H420FTG(PinName PwmA, PinName InA1, PinName InA2);
        void brake(void);
        void coast(void);
        void reset(void);
        void forward(float duty_cycle);
        void backward(float duty_cycle);
        float last_command;

    private:
        PinName InA1_;
        PinName InA2_;
        PinName PwmA_;
};

#endif