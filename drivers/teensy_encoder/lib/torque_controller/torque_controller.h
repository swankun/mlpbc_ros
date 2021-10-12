#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H

#include "Arduino.h"

class TorqueController
{
    public:
        TorqueController(float ampLimit=2.5f, float vcc=24.0f);

        void setGains(float p, float i);
        void setIntegralBounds(float i_min, float i_max);
        void setPGain(float p);
        void setIGain(float i);
        void getGains(float &p, float &i);
        void setAmpState(float amps);
        void setTargetTorque(float nm);
        void setTargetAmp(float amps);
        void getCurrentPIErrors(float &pe, float &ie);

        void resetIntegralError();
        float computeDutyCycle(unsigned int dt_us);

        struct MotorParams
        {
            float gear_ratio = (13.0f / 3.0f) * (45.0f / 34.0f);
            float eta = 0.88;      
            float k_tau = 52.5E-3f;              // N-m/a
            float k_emf = 1.0f/(182*2*M_PI/60);  // 182 rpm/V to V-s/rad
            float R = 2.03;                      // Ohms
            float L = 0.62E-3f;                  // Henry
        } motor_params;
        const float max_voltage;
        const float max_current;

    private:
        float p_gain_;
        float i_gain_;
        float state_torque_;
        float state_current_;
        float target_torque_;
        float target_current_;
        float p_error_;
        float i_error_;
        float i_min_;
        float i_max_;
        bool antiwindup_;
};


#endif // TORQUE_CONTROLLER_H


