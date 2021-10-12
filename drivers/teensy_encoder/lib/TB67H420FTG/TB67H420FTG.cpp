#include "TB67H420FTG.h"

// Helper functions for writing motor commands
template<class T>
T& map( T& x, T& in_min, T& in_max, T& out_min, T& out_max )
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

TB67H420FTG::TB67H420FTG(PinName PwmA, PinName InA1, PinName InA2) : 
    PwmA_(PwmA), InA1_(InA1), InA2_(InA2)
{
    last_command = 0.0f;
    analogWriteFrequency(PwmA, PWM_FREQUENCY_HZ);
    analogWriteResolution(PWM_RESOLUTION_BITS);
    this->reset();
}

void TB67H420FTG::brake()
{
  analogWrite(PwmA_, 0);
  digitalWriteFast(InA1_, HIGH);
  digitalWriteFast(InA2_, HIGH);
}

void TB67H420FTG::reset()
{
  analogWrite(PwmA_, 0);
  digitalWriteFast(InA1_, LOW);
  digitalWriteFast(InA2_, LOW);
}

void TB67H420FTG::coast()
{
  analogWrite(PwmA_, 10);
  digitalWriteFast(InA1_, LOW);
  digitalWriteFast(InA2_, LOW);
}

void TB67H420FTG::forward(float duty_cycle)
{
    if (0.0 < duty_cycle && duty_cycle <= 1.0) {
        digitalWriteFast(InA1_, LOW);
        digitalWriteFast(InA2_, HIGH);
        analogWrite(PwmA_, (int)map(duty_cycle, 0.0, 1.0, 0.0, 4095.0));
        last_command = duty_cycle;
    } else if (duty_cycle == 0.0) {
        this->coast();  
    } else {
        this->brake();
    }
}

void TB67H420FTG::backward(float duty_cycle)
{
    if (0.0 < duty_cycle && duty_cycle <= 1.0) {
        digitalWriteFast(InA1_, HIGH);
        digitalWriteFast(InA2_, LOW);
        analogWrite(PwmA_, (int)map(duty_cycle, 0.0, 1.0, 0.0, 4095.0));
        last_command = duty_cycle;
    } else if (duty_cycle == 0.0) {
        this->coast();  
    } else {
        this->brake();
    }
}
