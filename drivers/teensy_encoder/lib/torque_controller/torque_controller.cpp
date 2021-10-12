#include "torque_controller.h"

// Helper functions for writing motor commands
template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi )
{
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

TorqueController::TorqueController(float ampLimit, float vcc) :
    max_voltage(vcc),
    max_current(ampLimit),
    p_gain_(0.0),
    i_gain_(0.0),
    state_current_(0.0),
    target_torque_(0.0),
    target_current_(0.0),
    p_error_(0.0),
    i_error_(0.0),
    i_min_(-1.0),
    i_max_(1.0),
    antiwindup_(true)
{
}

void TorqueController::setGains(float p, float i)
{
    p_gain_ = p;
    i_gain_ = i;
}
void TorqueController::setIntegralBounds(float i_min, float i_max)
{
    i_min_ = i_min;
    i_max_ = i_max;
}
void TorqueController::setPGain(float p)
{
    p_gain_ = p;
}
void TorqueController::setIGain(float i)
{
    i_gain_ = i;
}
void TorqueController::getGains(float &p, float &i)
{
    p = p_gain_;
    i = i_gain_;
}

void TorqueController::setAmpState(float amps)
{
    state_current_ = amps;
    state_torque_  = amps * motor_params.gear_ratio * motor_params.eta * motor_params.k_tau;
}

void TorqueController::setTargetTorque(float nm)
{
    if (target_torque_ != nm)
    {
        target_torque_  = nm;
        target_current_ = nm / motor_params.gear_ratio / motor_params.eta / motor_params.k_tau;
        // resetIntegralError();
    }
}
void TorqueController::setTargetAmp(float amps)
{
    if (target_current_ != amps)
    {
        target_torque_  = amps * motor_params.gear_ratio * motor_params.eta * motor_params.k_tau;
        target_current_ = amps;
        // resetIntegralError();
    }
}


void TorqueController::getCurrentPIErrors(float &pe, float &ie)
{
    pe = p_error_;
    ie = i_error_;
}


void TorqueController::resetIntegralError()
{
    i_error_ = 0.0;
}

float TorqueController::computeDutyCycle(unsigned int dt_us)
{
    if (target_current_ == 0.0)
        return 0.0;

    float error = clamp(target_current_, -max_current, max_current) - state_current_;
    
    if (dt_us == 0 || isnan(error) || isinf(error))
        return 0.0;

    p_error_ = error;
    i_error_ += dt_us*1E-6f * p_error_;

    if (antiwindup_)
    {
        // Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
        i_error_ = clamp(i_error_, i_min_, i_max_);
    }

    float p_term, i_term, feedforward;
    p_term = p_gain_ * p_error_;
    i_term = i_gain_ * i_error_;
    if(!antiwindup_)
    {
        // Limit i_term so that the limit is meaningful in the output
        i_term = clamp(i_term, i_min_, i_max_);
    }
    feedforward = target_current_ * motor_params.R;

    float duty_cycle = clamp(p_term+i_term+feedforward, -max_voltage, max_voltage) / max_voltage;

    return duty_cycle;
}