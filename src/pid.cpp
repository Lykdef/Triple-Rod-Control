#include "triple/pid.hpp"
#include <algorithm>

PID::PID(
    double proportionalGain,
    double integralGain,
    double derivativeGain,
    double integralLimit):
        kp_(proportionalGain),
        ki_(integralGain),
        kd_(derivativeGain),
        integralLimit_(integralLimit)
{
}

double PID::update(
    double error,
    double errorRate,
    double dt)
{
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integralLimit_, integralLimit_);
    return kp_ * error + ki_ * integral_ + kd_ * errorRate;
}

void PID::reset() {
    integral_ = 0.0;
}