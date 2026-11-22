#pragma once

class PID {
public:
    PID(
        double proportionalGain,
        double integralGain,
        double derivativeGain,
        double integralLimit);
    
    double update(
        double error,
        double errorRate,
        double dt);
    
    void reset();

private:
    double kp_;
    double ki_;
    double kd_;

    double integralLimit_;
    double integral_ = 0.0;
};